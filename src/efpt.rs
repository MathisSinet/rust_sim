fn ef_pub_tables() -> Result<(), Box<dyn Error>> {
    let path = Path::new("data/ef.json");
    
    const CTEND: u32 = 375 * 32;
    const POINTS: [f64; 14] = [10., 20., 30., 40., 50., 70., 90., 110., 130., 150., 250., 275., 300., 325.];

    let content = fs::read_to_string(path)?;
    
    let mut pub_data: HashMap<u32, PubData> = serde_json::from_str(&content)?;

    println!("Read");
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


    for start in ((270*32)..(275*32)).rev() {
        println!("Starting pub tables for {}", start as f64 / 32.);
        println!("");

        for point in POINTS.iter() {
            if start as f64 / 32. < *point {
                next_milestone_cost = *point;
                break;
            }
        }
        best_t = f64::MAX;
        best_next = 0;
        a = if start >= 15*32 { 8.min(CTEND - start) } else { 1i32.max(10*32 - start as i32) as u32 };
        b = if start >= 15*32 { 200.min(CTEND - start) } else { 20*32 - start };

        simbase = ef::EF::new(
                TheoryData { 
                    tau: start as f64 * 1.6 / 32., 
                    students: 0, 
                    rho: 0.
                },
                (start + a) as f64 / 32. - 2.,
                None
            );
        simbase.efdata.do_coasting = false;

        for end in (start+a)..(start+b+1) {
            print!("\rTesting sim {}/{}", end - (start+a) + 1, b - a + 1);
            std::io::Write::flush(&mut std::io::stdout())?;

            simbase.goal = (end as f64 / 32.).min(next_milestone_cost) - 2.;
            simbase.simulate();

            sim = simbase.fork();
            sim.efdata.do_coasting = true;
            sim.goal = end as f64 / 32.;
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