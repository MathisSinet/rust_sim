mod utils;
mod t2;
mod t6;
mod csr2;
use crate::utils::*;
use crate::t2::*;
use crate::t6::*;
use crate::csr2::*;

use std::collections::HashMap;
//use std::hash::Hash;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

#[derive(Debug, Serialize, Deserialize)]
struct PubData {
    next: u32,
    t: f64
}

fn sim_t2() -> T2 {
    let sim: T2 = T2::new(
        TheoryData { 
            tau: strtolog10(s!("1e690")), 
            students: 460, 
            rho: 0.
        },
        strtolog10(s!("1e710")),
        None
    );

    sim
}

fn sim_t6() -> T6 {
    let sim: T6 = T6::new(
        TheoryData { 
            tau: strtolog10(s!("1.4e1182")), 
            students: 460, 
            rho: strtolog10(s!("6.5e1184"))
        },
        strtolog10(s!("1.08e1188")),
        Some(T6state {
            levels: [2483, 592, 233, 116, 3907, 1683, 2005],
            q: strtolog10(s!("4.23e258")),
            r: strtolog10(s!("1.355e45"))
        })
    );

    sim
}

fn sim_t6_base() -> T6 {
    T6::new(
        TheoryData { 
            tau: strtolog10(s!("1.4e1182")), 
            students: 460, 
            rho: strtolog10(s!("1e1183"))
        },
        strtolog10(s!("1.08e1188")),
        None
    )
}

fn sim_csr2() -> CSR2 {
    CSR2::new(
        TheoryData {
            tau: strtolog10(s!("1e1400")) * 0.4,
            students: 460,
            rho: 0.
        },
        strtolog10(s!("4.05e1401")),
        None
    )
}

fn csr2_pub_tables() -> Result<(), Box<dyn std::error::Error>> {
    let path = Path::new("data/csr2.json");
    
    const CTEND: u32 = 1500 * 16;

    let content = fs::read_to_string(path)?;
    
    let mut pub_data: HashMap<u32, PubData> = serde_json::from_str(&content)?;

    println!("Read");
    //println!("{:?}", pub_data.get(&24000));

    let mut best_t: f64;
    let mut best_next: u32;
    let mut sim: CSR2;
    let mut simbase: CSR2;
    let mut simt: f64;
    let mut a: u32;
    let mut b: u32;
    let mut end_t: f64;


    for start in ((900*16)..(1000*16)).rev() {
        println!("Starting pub tables for {}", start as f64 / 16.);
        best_t = std::f64::MAX;
        best_next = 0;
        a = 8.min(CTEND - start);
        b = 80.min(CTEND - start);

        simbase = CSR2::new(
                TheoryData { 
                    tau: start as f64 * 0.4 / 16., 
                    students: 0, 
                    rho: 0.
                },
                (start + a) as f64 / 16. - 1.8,
                None
            );
        simbase.csr2data.do_coasting = false;

        for end in (start+a)..(start+b+1) {
            simbase.goal = end as f64 / 16. - 1.8;
            simbase.simulate();

            sim = simbase.fork();
            sim.csr2data.do_coasting = true;
            sim.goal = end as f64 / 16.;
            simt = sim.simulate().t;

            end_t = match pub_data.get(&end) {
                    None => 1e100,
                    Some(pdata) => pdata.t
                };

            if simt + end_t < best_t {
                best_t = simt + end_t;
                best_next = end;
            }
        }

        pub_data.insert(start, PubData { next: best_next, t: best_t });
    }

    //pub_data.insert(23999, PubData { next: 24000, t: 0.1 });

    let updated_content = serde_json::to_string_pretty(&pub_data)?;
    fs::write(path, updated_content)?;

    Ok(())
}

fn csr2_pub_tables_read(rho: Option<f64>) -> Result<(), Box<dyn std::error::Error>> {
    let path = Path::new("data/csr2.json");
    
    const CTEND: u32 = 1500 * 16;

    let content = fs::read_to_string(path)?;
    
    let mut pub_data: HashMap<u32, PubData> = serde_json::from_str(&content)?;
    //let mut cur_data: Option<PubData>;

    match rho {
        None => for r in (1450*16)..(1500*16+1) {
            let curdata= pub_data.get(&r);
            match curdata {
                None => println!("No entry for {}", r),
                Some(data) => println!("{} -> {}; {} remains", r as f64 / 16., data.next as f64 / 16., get_time_string(data.t))
            }
        },
        Some(r) => {
            let a = (r * 16.).round() as u32;
            let curdata= pub_data.get(&a);
            match curdata {
                None => println!("No entry for {}", a as f64 / 16.),
                Some(data) => println!("{} -> {}; {} remains", a as f64 / 16., data.next as f64 / 16., get_time_string(data.t))
            }
        }
    }

    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {

    let _res = csr2_pub_tables()?;
    /*let _ = csr2_pub_tables_read(Some(900.)); // 10d 14h
    let _ = csr2_pub_tables_read(Some(950.)); // 10d 11h
    let _ = csr2_pub_tables_read(Some(1000.)); // 10d 9h
    let _ = csr2_pub_tables_read(Some(1050.)); // 10d 4h
    let _ = csr2_pub_tables_read(Some(1100.)); // 10d 5h
    let _ = csr2_pub_tables_read(Some(1150.)); // 9d 18h
    let _ = csr2_pub_tables_read(Some(1200.)); // 9d 5h
    let _ = csr2_pub_tables_read(Some(1250.)); // 8d 20h
    let _ = csr2_pub_tables_read(Some(1300.)); // 8d 8h
    let _ = csr2_pub_tables_read(Some(1350.)); // 7d 3h
    let _ = csr2_pub_tables_read(Some(1400.)); // 6d 3h
    let _ = csr2_pub_tables_read(Some(1450.)); // 3d 12h*/
    
    /*let mut sim = sim_t6();

    let res: SimRes = sim.simulate();

    println!("{:?}", res.var_buys);
    println!("{}", get_time_string(res.t));*/

    Ok(())
}
