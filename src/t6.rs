use crate::utils::*;

struct T6vars {
    q1: Variable<FirstFreeCost<ExponentialCost>, StepwiseValue>,
    q2: Variable<ExponentialCost, ExponentialValue>,
    r1: Variable<ExponentialCost, StepwiseValue>,
    r2: Variable<ExponentialCost, ExponentialValue>,
    c1: Variable<ExponentialCost, StepwiseValue>,
    c2: Variable<ExponentialCost, ExponentialValue>,
    c5: Variable<ExponentialCost, ExponentialValue>,
}

impl T6vars {
    fn init() -> Self {
        T6vars {
            q1: Variable::new(
                FirstFreeCost {
                    model: ExponentialCost::new(15., 3.),
                },
                StepwiseValue::new(2., 10),
            ), // q1
            q2: Variable::new(ExponentialCost::new(500., 100.), ExponentialValue::new(2.)), // q2
            r1: Variable::new(ExponentialCost::new(1e25, 1e5), StepwiseValue::new(2., 10)), // r1
            r2: Variable::new(ExponentialCost::new(1e30, 1e10), ExponentialValue::new(2.)), // r2
            c1: Variable::new(ExponentialCost::new(10., 2.), StepwiseValue::new(2., 10)),   // c1
            c2: Variable::new(ExponentialCost::new(100., 5.), ExponentialValue::new(2.)),   // c2
            c5: Variable::new(ExponentialCost::new(15., 3.9), ExponentialValue::new(2.)),   // c5
        }
    }

    fn _iter_noc12(&mut self) -> Vec<&mut dyn VariableTrait> {
        vec![
            &mut self.q1,
            &mut self.q2,
            &mut self.r1,
            &mut self.r2,
            &mut self.c5,
        ]
    }

    fn _iter_noc12_r(&mut self) -> Vec<&mut dyn VariableTrait> {
        vec![
            &mut self.c5,
            &mut self.r2,
            &mut self.r1,
            &mut self.q2,
            &mut self.q1,
        ]
    }

    fn getm(&mut self, id: usize) -> &mut dyn VariableTrait {
        match id {
            0 => &mut self.q1,
            1 => &mut self.q2,
            2 => &mut self.r1,
            3 => &mut self.r2,
            4 => &mut self.c1,
            5 => &mut self.c2,
            _ => &mut self.c5,
        }
    }

    fn get(&self, id: usize) -> &dyn VariableTrait {
        match id {
            0 => &self.q1,
            1 => &self.q2,
            2 => &self.r1,
            3 => &self.r2,
            4 => &self.c1,
            5 => &self.c2,
            _ => &self.c5,
        }
    }

    fn set(&mut self, lvls: [u32; 7]) {
        for (i, level) in lvls.iter().enumerate() {
            self.getm(i).set(*level);
        }
    }
}

#[derive(Clone)]
struct T6data {
    caps: [u32; 7],
    skip: [bool; 7],
    scale_start: f64,
    scale_end: f64,
    tol: f64,
}

impl Copy for T6data {}

pub struct T6state {
    pub levels: [u32; 7],
    pub q: f64,
    pub r: f64,
    pub tol: f64,
}

pub struct T6 {
    data: TheoryData,
    t6data: T6data,
    goal: f64,
    rho: f64,
    maxrho: f64,
    multiplier: f64,
    q: f64,
    r: f64,
    vars: T6vars,
    varbuys: Vec<VarBuy>,

    t: f64,
    dt: f64,
    ddt: f64,
    depth: u32,

    best_res: SimRes,
}

impl T6 {
    pub fn new(data: TheoryData, goal: f64, state: Option<T6state>) -> Self {
        let mut t6: T6 = T6 {
            data: data,
            t6data: T6data {
                caps: [u32::MAX; 7],
                skip: [false; 7],
                scale_start: -2.,
                scale_end: 2.,
                tol: 1.,
            },
            goal: goal,
            rho: 0.,
            maxrho: 0.,
            multiplier: 0.,
            q: 0.,
            r: 0.,
            vars: T6vars::init(),

            varbuys: Vec::new(),

            t: 0.,
            dt: 1.5,
            ddt: 1.0001,
            depth: 0,

            best_res: SimRes::default(),
        };

        if let Some(state) = state {
            t6.vars.set(state.levels);
            t6.q = state.q;
            t6.r = state.r;
            t6.t6data.tol = state.tol;
        }

        t6.rho = t6.data.rho;
        t6.multiplier = t6.get_multiplier(t6.data.tau, t6.data.students);

        t6
    }

    fn fork(&self) -> Self {
        let mut new = T6 {
            data: self.data,
            t6data: self.t6data,
            goal: self.goal,
            rho: self.rho,
            maxrho: self.maxrho,
            multiplier: self.multiplier,
            q: self.q,
            r: self.r,
            vars: T6vars::init(),

            varbuys: self.varbuys.clone(),
            t: self.t,
            dt: self.dt,
            ddt: self.ddt,
            depth: self.depth + 1,

            best_res: SimRes::default(),
        };

        for i in 0..7 {
            new.vars.getm(i).set(self.vars.get(i).get_level())
        }

        /*new.vars.q1.set(self.vars.q1.level);
        new.vars.q2.set(self.vars.q2.level);
        new.vars.r1.set(self.vars.r1.level);
        new.vars.r2.set(self.vars.r2.level);
        new.vars.c5.set(self.vars.c5.level);*/

        new
    }

    fn eval_coast(&self, id: usize, cost: f64) -> BuyEval {
        let dist: f64 = self.goal - cost;
        if dist > 2. {
            return BuyEval::BUY;
        }
        match id {
            0 | 2 => {
                if dist < 5f64.log10() {
                    BuyEval::SKIP
                } else {
                    BuyEval::FORK
                }
            }
            1 | 3 => {
                if dist < 2f64.log10() {
                    BuyEval::SKIP
                } else {
                    BuyEval::FORK
                }
            }
            6 => {
                if dist < 1.5f64.log10() {
                    BuyEval::SKIP
                } else if dist < 2.5f64.log10() {
                    BuyEval::FORK
                } else {
                    BuyEval::BUY
                }
            }
            _ => BuyEval::SKIP,
        }
    }

    fn eval_ratio_one(&self, ratio: f64, lbound: f64, ubound: f64) -> BuyEval {
        if ratio >= ubound {
            BuyEval::BUY
        } else if ratio > lbound {
            BuyEval::FORK
        } else {
            BuyEval::SKIP
        }
    }

    fn eval_ratio(&self, id: usize) -> BuyEval {
        let prog: f64 = self.get_pub_progress();
        let tol: f64 = self.t6data.tol * prog;
        let mut best_eval: BuyEval = BuyEval::BUY;
        let mut cur_eval: BuyEval;
        let mut base_ratio: f64;

        match id {
            0 => {
                let mod10: f64 = (self.vars.q1.level % 10) as f64;
                for cmp in [1, 3, 6] {
                    cur_eval = match cmp {
                        1 => {
                            base_ratio = (7. + mod10).log10();
                            self.eval_ratio_one(
                                self.vars.q2.cost - self.vars.q1.cost,
                                (base_ratio - 0.1 * tol).max(0.0),
                                base_ratio + 0.25 * tol,
                            )
                        }
                        3 => {
                            base_ratio = (7. + mod10).log10();
                            self.eval_ratio_one(
                                self.vars.r2.cost - self.vars.q1.cost,
                                (base_ratio - 0.05 * tol).max(0.0),
                                base_ratio + 0.6 * tol,
                            )
                        }
                        6 => {
                            base_ratio = (5. + 0.5 * mod10).log10();
                            self.eval_ratio_one(
                                self.vars.c5.cost - self.vars.q1.cost,
                                (base_ratio - (0.4 - 0.3 * prog.powi(2)) * tol).max(0.0),
                                base_ratio + (0.35 * prog.powi(2)) * tol,
                            )
                        }
                        _ => BuyEval::BUY,
                    };
                    if cur_eval == BuyEval::SKIP {
                        return BuyEval::SKIP;
                    }
                    if cur_eval == BuyEval::FORK {
                        best_eval = BuyEval::FORK;
                    }
                }
            }
            1 => {
                for cmp in [3, 6] {
                    cur_eval = match cmp {
                        3 => {
                            base_ratio = 0.1;
                            self.eval_ratio_one(
                                self.vars.r2.cost - self.vars.q2.cost,
                                base_ratio - 0.1 * tol,
                                base_ratio + 0.2 * tol,
                            )
                        }
                        6 => {
                            if self.t6data.skip[6] || prog < 0.7 {
                                BuyEval::BUY
                            } else {
                                self.eval_ratio_one(
                                    self.vars.c5.cost - self.vars.q2.cost,
                                    0.,
                                    0.5 * tol * prog,
                                )
                            }
                        }
                        _ => BuyEval::BUY,
                    };
                    if cur_eval == BuyEval::SKIP {
                        return BuyEval::SKIP;
                    }
                    if cur_eval == BuyEval::FORK {
                        best_eval = BuyEval::FORK;
                    }
                }
            }
            2 => {
                let mod10: f64 = (self.vars.r1.level % 10) as f64;
                for cmp in [1, 3, 6] {
                    cur_eval = match cmp {
                        1 => {
                            base_ratio = (3. + 0.5 * mod10).log10();
                            self.eval_ratio_one(
                                self.vars.q2.cost - self.vars.r1.cost,
                                (base_ratio - 0.1 * tol).max(0.0),
                                base_ratio + 0.1 * tol,
                            )
                        }
                        3 => {
                            if self.vars.r2.cost + 1. > self.vars.r1.cost {
                                BuyEval::BUY
                            } else {
                                BuyEval::SKIP
                            }
                        }
                        6 => {
                            base_ratio = (3. + 0.5 * mod10).log10();
                            self.eval_ratio_one(
                                self.vars.c5.cost - self.vars.r1.cost,
                                (base_ratio - (0.25 + 0.25 * prog.powi(2)) * tol).max(0.0),
                                base_ratio + (0.25 * prog.powi(2)) * tol,
                            )
                        }
                        _ => BuyEval::BUY,
                    };
                    if cur_eval == BuyEval::SKIP {
                        return BuyEval::SKIP;
                    }
                    if cur_eval == BuyEval::FORK {
                        best_eval = BuyEval::FORK;
                    }
                }
            }
            3 => (),
            6 => {
                for cmp in [1, 3] {
                    cur_eval = match cmp {
                        1 => {
                            if self.t6data.skip[1] || prog > 0.7 {
                                BuyEval::BUY
                            } else {
                                self.eval_ratio_one(
                                    self.vars.q2.cost - self.vars.c5.cost,
                                    0.,
                                    0.15 * tol,
                                )
                            }
                        }
                        3 => self.eval_ratio_one(
                            self.vars.r2.cost - self.vars.c5.cost,
                            0.,
                            0.3 * tol,
                        ),
                        _ => BuyEval::BUY,
                    };
                    if cur_eval == BuyEval::SKIP {
                        return BuyEval::SKIP;
                    }
                    if cur_eval == BuyEval::FORK {
                        best_eval = BuyEval::FORK;
                    }
                }
            }
            _ => (),
        }

        best_eval
    }

    fn get_multiplier(&self, tau: f64, sigma: u32) -> f64 {
        3. * (sigma as f64 / 20.).log10() + 0.196 * tau - 50f64.log10()
    }

    fn get_pub_progress(&self) -> f64 {
        let rho_start: f64 = self.data.tau + self.t6data.scale_start;
        let rho_end: f64 = self.goal - self.t6data.scale_end;

        if self.maxrho <= rho_start {
            0.
        } else if self.maxrho >= rho_end {
            1.
        } else {
            (self.maxrho - rho_start) / (rho_end - rho_start)
        }
    }

    fn calc_integral(&self) -> f64 {
        let term1: f64 = self.vars.c1.value * 1.15 + self.vars.c2.value + self.q + self.r;
        let term2: f64 = self.vars.c5.value + self.q + 2. * self.r - 2f64.log10();

        self.multiplier + log10add(term1, term2)
    }

    fn tick(&mut self) {
        let logdt: f64 = self.dt.log10();

        let mut c: f64 = log10sub(self.calc_integral(), self.rho);

        self.q = log10add(self.q, self.vars.q1.value + self.vars.q2.value + logdt);
        self.r = log10add(self.r, self.vars.r1.value + self.vars.r2.value + logdt - 3.);

        let newrho: f64 = self.calc_integral();

        c = newrho.min(c);
        self.rho = log10sub(newrho, c);
        self.maxrho = self.maxrho.max(self.rho);

        self.t += self.dt / 1.5;
        self.dt *= self.ddt;
    }

    pub fn simulate(&mut self) -> SimRes {
        while self.maxrho < self.goal {
            self.tick();
            self.buy();
        }
        //println!("{};{};{};{};{}", self.vars.q1.level, self.vars.q2.level, self.vars.r1.level, self.vars.r2.level, self.vars.c5.level);

        if self.t < self.best_res.t {
            SimRes {
                t: self.t,
                var_buys: Some(self.varbuys.clone()),
            }
        } else {
            SimRes {
                t: self.best_res.t,
                var_buys: self.best_res.var_buys.clone(),
            }
        }
    }

    fn _buy_noc1234(&mut self) {
        let mut cost: f64;
        let variables = [(6, "c5"), (3, "r2"), (2, "r1"), (1, "q2"), (0, "q1")];

        for (id, name) in variables {
            cost = self.vars.get(id).get_cost();
            while self.rho > cost {
                self.rho = log10sub(self.rho, cost);
                self.vars.getm(id).buy();
                if self.rho > self.data.tau - 5. {
                    self.varbuys.push(VarBuy {
                        symb: String::from(name),
                        lvl: self.vars.get(id).get_level(),
                        t: self.t,
                    });
                }
                cost = self.vars.get(id).get_cost();
            }
        }
    }

    fn buy(&mut self) {
        let mut cost: f64;
        let mut coast_eval: BuyEval;
        let mut ratio_eval: BuyEval;
        let variables: [(usize, &str); 5] = [(6, "c5"), (3, "r2"), (2, "r1"), (1, "q2"), (0, "q1")];

        for (id, name) in variables {
            if self.t6data.skip[id] {
                continue;
            }
            if self.vars.get(id).get_level() >= self.t6data.caps[id] {
                continue;
            }

            cost = self.vars.get(id).get_cost();

            while self.rho > cost {
                coast_eval = self.eval_coast(id, cost);
                ratio_eval = self.eval_ratio(id);

                if coast_eval != BuyEval::SKIP {
                    if ratio_eval == BuyEval::SKIP {
                        break;
                    }
                    if coast_eval == BuyEval::FORK {
                        let mut fork: T6 = self.fork();
                        let lvl: u32 = self.vars.get(id).get_level();
                        fork.t6data.caps[id] = lvl;
                        if self.depth <= 3 {
                            println!(
                                "Depth {}; Creating coasting fork for {} lvl {}",
                                self.depth, name, lvl
                            );
                        }
                        let res: SimRes = fork.simulate();
                        if self.depth <= 3 {
                            //println!("Finished the fork!")
                        }
                        if res.t < self.best_res.t {
                            self.best_res.t = res.t;
                            self.best_res.var_buys = res.var_buys;
                        }
                    }
                    if ratio_eval == BuyEval::FORK {
                        let mut fork: T6 = self.fork();
                        fork.t6data.skip[id] = true;
                        if self.depth <= 3 {
                            println!(
                                "Depth {}; Creating ratio fork for {} lvl {}",
                                self.depth,
                                name,
                                self.vars.get(id).get_level()
                            );
                        }
                        let res: SimRes = fork.simulate();
                        if res.t < self.best_res.t {
                            self.best_res.t = res.t;
                            self.best_res.var_buys = res.var_buys;
                        }
                    }

                    self.rho = log10sub(self.rho, cost);
                    self.vars.getm(id).buy();
                    cost = self.vars.get(id).get_cost();
                    self.t6data.skip.fill(false);

                    if self.maxrho > self.data.tau - 5. {
                        self.varbuys.push(VarBuy {
                            symb: name.to_string(),
                            lvl: self.vars.get(id).get_level(),
                            t: self.t,
                        })
                    }
                } else {
                    self.t6data.caps[id] = self.vars.get(id).get_level();
                    break;
                }
            }
        }
    }
}
