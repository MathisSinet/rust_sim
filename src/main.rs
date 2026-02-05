mod csr2;
mod de;
mod ef;
mod fp;
mod ioutils;
mod t1;
mod t2;
mod t6;
mod t7;
pub mod utils;
use ioutils::*;
use utils::*;

use std::error::Error;
use std::fs;
use std::io;
use std::path::Path;
use std::{collections::HashMap, fmt::Display};

//use serde::de;
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
struct PubData {
    next: u32,
    t: f64,
}

fn input_theory_data() -> Result<TheoryData, Box<dyn Error>> {
    Ok(TheoryData {
        students: input("Input students: ")?.parse::<u32>()?,
        tau: strtolog10(&input("Input tau: ")?),
        rho: strtolog10(&input("Input rho: ")?),
    })
}

fn input_levels<const N: usize>(names: &[impl Display; N]) -> Result<[u32; N], Box<dyn Error>> {
    let mut levels: [u32; N] = [0u32; N];

    for i in 0..N {
        levels[i] = input(&format!("Input {} level: ", names[i]))?.parse()?;
    }

    Ok(levels)
}

fn compress_pub_tables(source: String, dest: String) -> Result<(), Box<dyn Error>> {
    let source_path = Path::new(&source);
    let dest_path = Path::new(&dest);

    let content = fs::read_to_string(source_path)?;
    let pub_table: HashMap<u32, PubData> = serde_json::from_str(&content)?;
    let mut dest_data: HashMap<u32, u32> = HashMap::new();

    for key in pub_table.keys() {
        dest_data.insert(
            *key,
            match pub_table.get(key) {
                None => 0,
                Some(data) => data.next,
            },
        );
    }

    let updated_content = serde_json::to_string_pretty(&dest_data)?;
    fs::write(dest_path, updated_content)?;

    Ok(())
}

fn get_pub_tables_range(source: String, end: u32) -> Result<(), Box<dyn Error>> {
    let path = Path::new(&source);
    let content = fs::read_to_string(path)?;
    let pub_table: HashMap<u32, PubData> = serde_json::from_str(&content)?;
    let mut low = u32::MAX;
    let mut high = u32::MIN;
    let mut dist: u32;

    for (key, val) in pub_table.iter() {
        if *key == end {
            continue;
        }
        dist = val.next - key;
        low = low.min(dist);
        high = high.max(dist);
    }

    println!("{low} {high}");

    Ok(())
}

fn get_pub_tables_diff(
    source: String,
    start: u32,
    end: u32,
    grid: f64,
) -> Result<(), Box<dyn Error>> {
    let path = Path::new(&source);
    let content = fs::read_to_string(path)?;
    let pub_table: HashMap<u32, PubData> = serde_json::from_str(&content)?;

    for i in start..=end {
        match pub_table.get(&i) {
            None => println!("Entry not found for {}", i as f64 / grid),
            Some(pubdata) => println!(
                "{} -> {} diff {}",
                i as f64 / grid,
                pubdata.next as f64 / grid,
                pubdata.next - i
            ),
        }
    }

    Ok(())
}

fn sim_t1() -> t1::T1 {
    let sim: t1::T1 = t1::T1::new(
        TheoryData {
            tau: strtolog10("1.02e628"),
            students: 460,
            rho: 0.,
        },
        strtolog10("1.4e630"),
        None,
    );

    sim
}

fn sim_t7() {
    let mut sim: t7::T7 = t7::T7::new(
        TheoryData {
            tau: strtolog10("1.83e618"),
            students: 501,
            rho: 0.,
        },
        strtolog10("5.84e620"),
        None,
    );

    let res = sim.simulate();

    println!("{}", get_time_string(res.t));
}

fn sim_t2_input() -> Result<t2::T2, Box<dyn Error>> {
    let theory_data = input_theory_data()?;
    let goal = strtolog10(&input("Input goal: ")?);

    let layer_names = ["q1", "q2", "q3", "q4", "r1", "r2", "r3", "r4"];
    let var_names = layer_names.map(|s| "d".to_owned() + s);

    let levels = input_levels(&var_names)?;
    let mut layers = [0f64; 8];

    for (i, name) in layer_names.iter().enumerate() {
        layers[i] = strtolog10(&input(&format!("Input layer {}: ", name))?);
    }

    let sim: t2::T2 = t2::T2::new(theory_data, goal, Some(t2::T2state { levels, layers }));

    Ok(sim)
}

fn sim_t6_input() -> Result<t6::T6, Box<dyn Error>> {
    let theory_data = input_theory_data()?;
    let goal = strtolog10(&input("Input goal: ")?);

    let levels = input_levels(&["q1", "q2", "r1", "r2", "c1", "c2", "c5"])?;

    let sim: t6::T6 = t6::T6::new(
        theory_data,
        goal,
        Some(t6::T6state {
            levels: levels,
            q: strtolog10(&input("Input q: ")?),
            r: strtolog10(&input("Input r: ")?),
            tol: input("Input tol: ")?.parse()?,
        }),
    );

    Ok(sim)
}

fn sim_ef() -> ef::EF {
    let sim: ef::EF = ef::EF::new(
        TheoryData {
            tau: 294.09375 * 1.6,
            students: 0,
            rho: 0.,
        },
        297.21875,
        None,
    );

    sim
}

fn sim_csr2(rho: f64) -> Result<csr2::CSR2, Box<dyn Error>> {
    let path = Path::new("data/csr2.json");
    let content = fs::read_to_string(path)?;
    let pub_table: HashMap<u32, PubData> = serde_json::from_str(&content)?;
    let seek = (rho * 16.).round() as u32;
    let goal: u32 = match pub_table.get(&seek) {
        None => {
            return Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                format!("No entry found for rho {}", rho),
            )));
        }
        Some(entry) => entry.next,
    };

    Ok(csr2::CSR2::new(
        TheoryData {
            tau: rho * 0.4,
            students: 0,
            rho: 0.,
        },
        (goal as f64) / 16.,
        None,
    ))
}

fn sim_fp() -> fp::FP {
    let sim: fp::FP = fp::FP::new(
        TheoryData {
            tau: strtolog10("1.6e1968") * 0.3,
            students: 0,
            rho: 0.,
        },
        strtolog10("1e2000"),
        None,
    );

    sim
}

fn sim_de() -> de::DE {
    let sim: de::DE = de::DE::new(
        TheoryData {
            tau: strtolog10("2.4e581") * 0.4,
            students: 0,
            rho: 0.,
        },
        strtolog10("1.65e586"),
        None,
    );

    sim
}

fn t1_pub_tables() -> Result<(), Box<dyn Error>> {
    let path = Path::new("data/t1c34.json");

    const CTEND: u32 = 900 * 32;

    let content = fs::read_to_string(path)?;

    let mut pub_data: HashMap<u32, PubData> = serde_json::from_str(&content)?;

    println!("Read");
    //println!("{:?}", pub_data.get(&24000));

    let mut best_t: f64;
    let mut best_next: u32;
    let mut sim: t1::T1;
    let mut simbase: t1::T1;
    let mut simt: f64;
    let mut a: u32;
    let mut b: u32;
    let mut end_t: f64;

    for start in ((800 * 32)..(900 * 32)).rev() {
        println!("Starting pub tables for {}", start as f64 / 32.);
        best_t = f64::MAX;
        best_next = 0;
        a = 40.min(CTEND - start);
        b = 150.min(CTEND - start);

        simbase = t1::T1::new(
            TheoryData {
                tau: start as f64 / 32.,
                students: 500,
                rho: 0.,
            },
            (start + a) as f64 / 32. - 6.,
            None,
        );
        simbase.t1data.do_coasting = false;

        for end in (start + a)..(start + b + 1) {
            simbase.goal = end as f64 / 32. - 6.;
            simbase.simulate();

            sim = simbase.fork();
            sim.t1data.do_coasting = true;
            sim.goal = end as f64 / 32.;
            simt = sim.simulate().t;

            end_t = match pub_data.get(&end) {
                None => 1e100,
                Some(pdata) => pdata.t,
            };

            if simt + end_t < best_t {
                best_t = simt + end_t;
                best_next = end;
            }
        }

        pub_data.insert(
            start,
            PubData {
                next: best_next,
                t: best_t,
            },
        );
    }

    //pub_data.insert(23999, PubData { next: 24000, t: 0.1 });

    let updated_content = serde_json::to_string_pretty(&pub_data)?;
    fs::write(path, updated_content)?;

    Ok(())
}

fn t7_pub_tables() -> Result<(), Box<dyn Error>> {
    let path = Path::new("data/t7.json");

    const CTEND: u32 = 800 * 32;

    let content = fs::read_to_string(path)?;

    let mut pub_data: HashMap<u32, PubData> = serde_json::from_str(&content)?;

    println!("Read");
    //println!("{:?}", pub_data.get(&24000));

    let mut best_t: f64;
    let mut best_next: u32;
    let mut sim: t7::T7;
    let mut simbase: t7::T7;
    let mut simt: f64;
    let mut a: u32;
    let mut b: u32;
    let mut end_t: f64;

    for start in ((700 * 32)..(800 * 32)).rev() {
        println!("Starting pub tables for {}", start as f64 / 32.);
        best_t = f64::MAX;
        best_next = 0;
        a = 40.min(CTEND - start);
        b = 128.min(CTEND - start);

        simbase = t7::T7::new(
            TheoryData {
                tau: start as f64 / 32.,
                students: 500,
                rho: 0.,
            },
            (start + a) as f64 / 32. - 1.5,
            None,
        );
        simbase.t7data.do_coasting = false;

        for end in (start + a)..(start + b + 1) {
            simbase.goal = end as f64 / 32. - 1.5;
            simbase.simulate();

            sim = simbase.fork();
            sim.t7data.do_coasting = true;
            sim.goal = end as f64 / 32.;
            simt = sim.simulate().t;

            end_t = match pub_data.get(&end) {
                None => 1e100,
                Some(pdata) => pdata.t,
            };

            if simt + end_t < best_t {
                best_t = simt + end_t;
                best_next = end;
            }
        }

        pub_data.insert(
            start,
            PubData {
                next: best_next,
                t: best_t,
            },
        );
    }

    //pub_data.insert(23999, PubData { next: 24000, t: 0.1 });

    let updated_content = serde_json::to_string_pretty(&pub_data)?;
    fs::write(path, updated_content)?;

    Ok(())
}

fn csr2_pub_tables() -> Result<(), Box<dyn Error>> {
    let path = Path::new("data/csr2.json");

    const CTEND: u32 = 1500 * 16;

    let content = fs::read_to_string(path)?;

    let mut pub_data: HashMap<u32, PubData> = serde_json::from_str(&content)?;

    println!("Read");
    //println!("{:?}", pub_data.get(&24000));

    let mut best_t: f64;
    let mut best_next: u32;
    let mut sim: csr2::CSR2;
    let mut simbase: csr2::CSR2;
    let mut simt: f64;
    let mut a: u32;
    let mut b: u32;
    let mut end_t: f64;

    for start in ((500 * 16)..(700 * 16)).rev() {
        println!("Starting pub tables for {}", start as f64 / 16.);
        best_t = f64::MAX;
        best_next = 0;
        a = 8.min(CTEND - start);
        b = 80.min(CTEND - start);

        simbase = csr2::CSR2::new(
            TheoryData {
                tau: start as f64 * 0.4 / 16.,
                students: 0,
                rho: 0.,
            },
            (start + a) as f64 / 16. - 1.8,
            None,
        );
        simbase.csr2data.do_coasting = false;

        for end in (start + a)..(start + b + 1) {
            simbase.goal = end as f64 / 16. - 1.8;
            simbase.simulate();

            sim = simbase.fork();
            sim.csr2data.do_coasting = true;
            sim.goal = end as f64 / 16.;
            simt = sim.simulate().t;

            end_t = match pub_data.get(&end) {
                None => 1e100,
                Some(pdata) => pdata.t,
            };

            if simt + end_t < best_t {
                best_t = simt + end_t;
                best_next = end;
            }
        }

        pub_data.insert(
            start,
            PubData {
                next: best_next,
                t: best_t,
            },
        );
    }

    //pub_data.insert(23999, PubData { next: 24000, t: 0.1 });

    let updated_content = serde_json::to_string_pretty(&pub_data)?;
    fs::write(path, updated_content)?;

    Ok(())
}

fn fp_pub_tables() -> Result<(), Box<dyn Error>> {
    let path = Path::new("data/fp.json");

    const CTEND: u32 = 2000 * 8;

    let content = fs::read_to_string(path)?;

    let mut pub_data: HashMap<u32, PubData> = serde_json::from_str(&content)?;

    println!("Read");
    //println!("{:?}", pub_data.get(&24000));

    let mut best_t: f64;
    let mut best_next: u32;
    let mut sim: fp::FP;
    let mut simbase: fp::FP;
    let mut simt: f64;
    let mut a: u32;
    let mut b: u32;
    let mut end_t: f64;

    for start in ((1200 * 8)..(1300 * 8)).rev() {
        println!("Starting pub tables for {}", start as f64 / 8.);
        best_t = f64::MAX;
        best_next = 0;
        a = 40.min(CTEND - start);
        b = 350.min(CTEND - start);

        simbase = fp::FP::new(
            TheoryData {
                tau: start as f64 * 0.3 / 8.,
                students: 0,
                rho: 0.,
            },
            (start + a) as f64 / 8. - 1.8,
            None,
        );
        simbase.fpdata.do_coasting = false;

        for end in (start + a)..(start + b + 1) {
            simbase.goal = end as f64 / 8. - 1.8;
            simbase.simulate();

            sim = simbase.fork();
            sim.fpdata.do_coasting = true;
            sim.goal = end as f64 / 8.;
            simt = sim.simulate().t;

            end_t = match pub_data.get(&end) {
                None => 1e100,
                Some(pdata) => pdata.t,
            };

            if simt + end_t < best_t {
                best_t = simt + end_t;
                best_next = end;
            }
        }

        pub_data.insert(
            start,
            PubData {
                next: best_next,
                t: best_t,
            },
        );
    }

    //pub_data.insert(23999, PubData { next: 24000, t: 0.1 });

    let updated_content = serde_json::to_string_pretty(&pub_data)?;
    fs::write(path, updated_content)?;

    Ok(())
}

fn de_pub_tables() -> Result<(), Box<dyn Error>> {
    let path = Path::new("data/de1050.json");

    const CTEND: u32 = 1050 * 16;

    let content = fs::read_to_string(path)?;

    let mut pub_data: HashMap<u32, PubData> = serde_json::from_str(&content)?;

    println!("Read");
    //println!("{:?}", pub_data.get(&24000));

    let mut best_t: f64;
    let mut best_next: u32;
    let mut sim: de::DE;
    let mut simbase: de::DE;
    let mut simt: f64;
    let mut a: u32;
    let mut b: u32;
    let mut end_t: f64;

    for start in ((900 * 16)..(950 * 16)).rev() {
        println!("Starting pub tables for {}", start as f64 / 16.);
        best_t = f64::MAX;
        best_next = 0;
        a = 8.min(CTEND - start);
        b = (6 * 16 + 8).min(CTEND - start);

        simbase = de::DE::new(
            TheoryData {
                tau: start as f64 * 0.4 / 16.,
                students: 0,
                rho: 0.,
            },
            (start + a) as f64 / 16. - 1.8,
            None,
        );
        simbase.dedata.do_coasting = false;

        for end in (start + a)..(start + b + 1) {
            print!("\rTesting sim {}/{}", end - (start + a) + 1, b - a + 1);
            io::Write::flush(&mut io::stdout())?;
            simbase.goal = end as f64 / 16. - 1.8;
            simbase.simulate();

            sim = simbase.fork();
            sim.dedata.do_coasting = true;
            sim.goal = end as f64 / 16.;
            simt = sim.simulate().t;

            end_t = match pub_data.get(&end) {
                None => 1e100,
                Some(pdata) => pdata.t,
            };

            if simt + end_t < best_t {
                best_t = simt + end_t;
                best_next = end;
            }
        }

        println!();
        println!(
            "Best next: {} ; Total time remaining: {}",
            best_next as f64 / 16.,
            get_time_string(best_t)
        );

        pub_data.insert(
            start,
            PubData {
                next: best_next,
                t: best_t,
            },
        );
    }

    //pub_data.insert(23999, PubData { next: 24000, t: 0.1 });

    let updated_content = serde_json::to_string_pretty(&pub_data)?;
    fs::write(path, updated_content)?;

    Ok(())
}

fn ef_pub_tables() -> Result<(), Box<dyn Error>> {
    let path = Path::new("data/ef.json");

    const CTEND: u32 = 375 * 32;
    const POINTS: [f64; 14] = [
        10., 20., 30., 40., 50., 70., 90., 110., 130., 150., 250., 275., 300., 325.,
    ];

    let content = fs::read_to_string(path)?;

    let mut pub_data: HashMap<u32, PubData> = serde_json::from_str(&content)?;

    println!("Read the file successfully.");
    //println!("{:?}", pub_data.get(&24000));

    let mut best_t: f64;
    let mut best_next: u32;
    let mut sim: ef::EF;
    let mut simbase: ef::EF;
    let mut simt: f64;
    let mut a: u32;
    let mut b: u32;
    let mut end_t: f64;
    let mut next_milestone_cost: f64 = f64::MAX;

    for start in (0..(5 * 32)).rev() {
        println!("Starting pub tables for {}", start as f64 / 32.);

        for point in POINTS.iter() {
            if start as f64 / 32. < *point {
                next_milestone_cost = *point;
                break;
            }
        }
        best_t = f64::MAX;
        best_next = 0;
        a = if start >= 15 * 32 {
            8.min(CTEND - start)
        } else {
            1i32.max(10 * 32 - start as i32) as u32
        };
        b = if start >= 15 * 32 {
            175.min(CTEND - start)
        } else {
            13 * 32 - start
        };

        simbase = ef::EF::new(
            TheoryData {
                tau: start as f64 * 1.6 / 32.,
                students: 0,
                rho: 0.,
            },
            (start + a) as f64 / 32. - 2.,
            None,
        );
        simbase.efdata.do_coasting = false;

        for end in (start + a)..=(start + b) {
            print!("\rTesting sim {}/{}", end - (start + a) + 1, b - a + 1);
            std::io::Write::flush(&mut std::io::stdout())?;

            simbase.goal = (end as f64 / 32.).min(next_milestone_cost) - 2.;
            simbase.simulate();

            sim = simbase.fork();
            sim.efdata.do_coasting = true;
            sim.goal = end as f64 / 32.;
            simt = sim.simulate().t;

            end_t = match pub_data.get(&end) {
                None => 1e100,
                Some(pdata) => pdata.t,
            };

            if simt + end_t < best_t {
                best_t = simt + end_t;
                best_next = end;
            }
        }

        println!();
        println!(
            "Best next: {} ; Total time remaining: {} ; Index diff: {}",
            best_next as f64 / 32.,
            get_time_string(best_t),
            best_next - start
        );

        pub_data.insert(
            start,
            PubData {
                next: best_next,
                t: best_t,
            },
        );
    }

    //pub_data.insert(23999, PubData { next: 24000, t: 0.1 });

    let updated_content = serde_json::to_string_pretty(&pub_data)?;
    fs::write(path, updated_content)?;

    Ok(())
}

fn pub_tables_read_chain(path: String, rho: f64, grid: f64) -> Result<(), Box<dyn Error>> {
    let path = Path::new(&path);
    let content = fs::read_to_string(path)?;
    let pub_table: HashMap<u32, PubData> = serde_json::from_str(&content)?;
    let mut index: u32 = (rho * grid).round() as u32;
    let mut curpubtime: f64;

    loop {
        match pub_table.get(&index) {
            None => {
                println!("No entry for {}", index as f64 / grid);
                break;
            }
            Some(data) => {
                curpubtime = match pub_table.get(&(data.next)) {
                    None => 0.,
                    Some(nextdata) => data.t - nextdata.t,
                };
                println!(
                    "{} -> {}; {}; current pub: {}, {:.3}",
                    log10tostr(index as f64 / grid),
                    log10tostr(data.next as f64 / grid),
                    get_time_string(data.t),
                    get_time_string(curpubtime),
                    10f64.powf((data.next as f64 / grid - index as f64 / grid) * 0.152)
                );
                if data.t > 0. {
                    index = data.next;
                } else {
                    break;
                }
            }
        }
    }

    Ok(())
}

fn rust_sim_cli() -> Result<(), Box<dyn Error>> {
    let mode_input = &input("Enter the mode : ")?[..];

    match mode_input {
        "T2" => {
            let mut sim = sim_t2_input()?;
            let res: SimRes = sim.simulate();
            println!("{:?}", res.var_buys);
            println!("{}", get_time_string(res.t));

            let defvec: Vec<VarBuy> = Vec::new();
            for variable in ["q1", "q2", "q3", "q4", "r1", "r2", "r3", "r4"] {
                match get_last_purchase(
                    match res.var_buys {
                        None => &defvec,
                        Some(ref varbuys) => varbuys,
                    },
                    variable,
                ) {
                    None => println!("Variable {variable} not found!"),
                    Some(level) => println!("Last purchase for {variable}: {level}"),
                }
            }
        }
        "T6" => {
            let mut sim = sim_t6_input()?;
            let res: SimRes = sim.simulate();
            println!("{:?}", res.var_buys);
            println!("{}", get_time_string(res.t));
        }
        _ => (),
    }

    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    //rust_sim_cli()?;
    //sim_t7();
    let _res = t1_pub_tables()?;
    //let _res2 = get_pub_tables_range(s!("data/ef.json"), 375*32);
    //let _ = get_pub_tables_diff(s!("data/de.json"), 875*16, 900*16, 16.);
    //let _ = pub_tables_read_chain(s!("data/de1050.json"), 900., 16.);
    //let _ = pub_tables_read_chain(s!("data/de1050.json"), (strtolog10(s!("4.97e143")) + 4f64.log10()) * 2.5 * 2.5, 16.);
    let _ = pub_tables_read_chain(s!("data/t1c34.json"), 800., 32.);

    //println!("{}", get_t(2291));

    //compress_pub_tables(s!("data/ef.json"), s!("data/EFpubtable.json"))?;

    Ok(())
}
