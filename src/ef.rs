use crate::s;
use crate::utils::*;

#[derive(Debug)]
struct EFvars {
    tdot: Variable<ExponentialCost, LinearValue>,
    q1: Variable<FirstFreeCost<ExponentialCost>, StepwiseValue>,
    q2: Variable<ExponentialCost, ExponentialValue>,
    b1: Variable<FirstFreeCost<ExponentialCost>, StepwiseValue>,
    b2: Variable<ExponentialCost, ExponentialValue>,
    c1: Variable<FirstFreeCost<ExponentialCost>, StepwiseValue>,
    c2: Variable<ExponentialCost, ExponentialValue>,
    a1: Variable<FirstFreeCost<ExponentialCost>, StepwiseValue>,
    a2: Variable<ExponentialCost, StepwiseValue>,
    a3: Variable<ExponentialCost, ExponentialValue>,
}

impl EFvars {
    fn init() -> Self {
        EFvars {
            tdot: Variable::new(ExponentialCost::new(1e6, 1e6), LinearValue::new(0.2, 0.2)),
            q1: Variable::new(
                FirstFreeCost {
                    model: ExponentialCost::new(10., 1.61328),
                },
                StepwiseValue::new(2., 10),
            ),
            q2: Variable::new(ExponentialCost::new(5., 60.), ExponentialValue::new(2.)),
            b1: Variable::new(
                FirstFreeCost {
                    model: ExponentialCost::new(20., 200.),
                },
                StepwiseValue::new_offset(2., 10, 1.),
            ),
            b2: Variable::new(ExponentialCost::new(100., 2.), ExponentialValue::new(1.1)),
            c1: Variable::new(
                FirstFreeCost {
                    model: ExponentialCost::new(20., 200.),
                },
                StepwiseValue::new_offset(2., 10, 1.),
            ),
            c2: Variable::new(ExponentialCost::new(100., 2.), ExponentialValue::new(1.1)),
            a1: Variable::new(
                FirstFreeCost {
                    model: ExponentialCost::new(2000., 2f64.powf(2.2)),
                },
                StepwiseValue::new_offset(2., 10, 1.),
            ),
            a2: Variable::new(
                ExponentialCost::new(500., 2f64.powf(2.2)),
                StepwiseValue::new_offset(40., 10, 1.),
            ),
            a3: Variable::new(
                ExponentialCost::new(500., 2f64.powf(2.2)),
                ExponentialValue::new(2.),
            ),
        }
    }

    fn update_b2_base(&mut self, base: f64) {
        let level = self.b2.level;
        self.b2 = Variable::new(ExponentialCost::new(100., 2.), ExponentialValue::new(base));
        self.b2.set(level);
    }

    fn update_c2_base(&mut self, base: f64) {
        let level = self.c2.level;
        self.c2 = Variable::new(ExponentialCost::new(100., 2.), ExponentialValue::new(base));
        self.c2.set(level);
    }

    fn getm(&mut self, id: usize) -> &mut dyn VariableTrait {
        match id {
            0 => &mut self.tdot,
            1 => &mut self.q1,
            2 => &mut self.q2,
            3 => &mut self.b1,
            4 => &mut self.b2,
            5 => &mut self.c1,
            6 => &mut self.c2,
            7 => &mut self.a1,
            8 => &mut self.a2,
            9 | _ => &mut self.a3,
        }
    }

    fn get(&self, id: usize) -> &dyn VariableTrait {
        match id {
            0 => &self.tdot,
            1 => &self.q1,
            2 => &self.q2,
            3 => &self.b1,
            4 => &self.b2,
            5 => &self.c1,
            6 => &self.c2,
            7 => &self.a1,
            8 => &self.a2,
            9 | _ => &self.a3,
        }
    }

    fn set(&mut self, lvls: [u32; 10]) {
        for i in 0..10 {
            self.getm(i).set(lvls[i]);
        }
    }
}

#[derive(Clone, Debug)]
pub struct EFdata {
    caps: [u32; 10],
    pub do_coasting: bool,
}

impl Copy for EFdata {}

pub struct EFstate {
    pub levels: [u32; 10],
    pub re: f64,
    pub im: f64,
    pub tvar: f64,
    pub q: f64,
}

#[derive(Debug)]
pub struct EF {
    data: TheoryData,
    pub efdata: EFdata,
    pub goal: f64,
    rho: f64,
    re: f64,
    im: f64,
    maxrho: f64,
    multiplier: f64,
    tvar: f64,
    q: f64,
    vars: EFvars,
    varbuys: Vec<VarBuy>,
    milestones: [usize; 5],
    next_milestone_cost: f64,

    t: f64,
    dt: f64,
    ddt: f64,
    depth: u32,

    best_res: SimRes,
}

impl EF {
    pub fn new(data: TheoryData, goal: f64, state: Option<EFstate>) -> Self {
        let mut ef: EF = EF {
            data: data,
            efdata: EFdata {
                caps: [u32::MAX; 10],
                do_coasting: true,
            },
            goal: goal,

            rho: 0.,
            re: 0.,
            im: 0.,
            maxrho: 0.,
            multiplier: 0.,
            tvar: 0.,
            q: 0.,
            vars: EFvars::init(),
            varbuys: Vec::new(),
            milestones: [0; 5],
            next_milestone_cost: f64::MAX,

            t: 0.,
            dt: 1.5,
            ddt: 1.0001,
            depth: 0,

            best_res: SimRes::default(),
        };

        match state {
            None => (),
            Some(state) => {
                ef.vars.set(state.levels);
                ef.re = state.re;
                ef.im = state.im;
                ef.tvar = state.tvar;
                ef.q = state.q;
            }
        };

        ef.rho = ef.data.rho;
        ef.multiplier = ef.get_multiplier(ef.data.tau);
        ef.update_milestones();

        ef
    }

    pub fn fork(&self) -> Self {
        let mut new: EF = EF {
            data: self.data,
            efdata: self.efdata,
            goal: self.goal,
            rho: self.rho,
            re: self.re,
            im: self.im,
            maxrho: self.maxrho,
            multiplier: self.multiplier,
            tvar: self.tvar,
            q: self.q,
            vars: EFvars::init(),
            varbuys: self.varbuys.clone(),
            milestones: self.milestones,
            next_milestone_cost: self.next_milestone_cost,
            t: self.t,
            dt: self.dt,
            ddt: self.ddt,
            depth: self.depth + 1,

            best_res: SimRes::default(),
        };

        for i in 0..10 {
            new.vars.getm(i).set(self.vars.get(i).get_level())
        }
        new.vars
            .update_b2_base(1.1 + 0.01 * new.milestones[3] as f64);
        new.vars
            .update_c2_base(1.1 + 0.0125 * new.milestones[4] as f64);

        new
    }

    fn get_multiplier(&self, tau: f64) -> f64 {
        tau * 0.09675
    }

    fn update_milestones(&mut self) {
        const POINTS: [f64; 14] = [
            10., 20., 30., 40., 50., 70., 90., 110., 130., 150., 250., 275., 300., 325.,
        ];
        const MAX_MS: [usize; 5] = [2, 3, 5, 2, 2];
        let rho = self.maxrho.max(self.data.tau * (1. / 1.6));
        let ms_b2 = self.milestones[3];
        let ms_c2 = self.milestones[4];
        self.milestones = [0; 5];
        let mut stage: u32 = 0;

        for point in &POINTS {
            if rho >= *point {
                stage += 1;
            } else {
                self.next_milestone_cost = *point;
                break;
            }
        }
        if stage == 14 {
            self.next_milestone_cost = f64::MAX
        }

        for i in 0..5 {
            while self.milestones[i] < MAX_MS[i] && stage > 0 {
                self.milestones[i] += 1;
                stage -= 1;
            }
        }

        if self.milestones[3] > ms_b2 {
            self.vars
                .update_b2_base(1.1 + 0.01 * self.milestones[3] as f64);
        }
        if self.milestones[4] > ms_c2 {
            self.vars
                .update_c2_base(1.1 + 0.0125 * self.milestones[4] as f64);
        }
    }

    fn eval_coast_one(&self, dist: f64, lbound: f64, ubound: f64) -> BuyEval {
        if dist > ubound {
            BuyEval::BUY
        } else if dist > lbound {
            BuyEval::FORK
        } else {
            BuyEval::SKIP
        }
    }

    fn eval_coast(&self, id: usize, cost: f64) -> BuyEval {
        //return if id >= 8 || self.maxrho + 5f64.log10() < self.next_milestone_cost { BuyEval::BUY } else { BuyEval::SKIP };
        let dist: f64 = self.goal.min(self.next_milestone_cost) - cost;
        if dist > 3. || !self.efdata.do_coasting {
            return BuyEval::BUY;
        }
        match id {
            1 => self.eval_coast_one(dist, 0.6, 1.8),
            2 => self.eval_coast_one(dist, 0.2, 1.5),
            7 => self.eval_coast_one(dist, 0.3, 1.5),
            _ => BuyEval::BUY,
        }
    }

    fn eval_ratio(&self, id: usize) -> BuyEval {
        //return BuyEval::BUY;
        let log10_5: f64 = 5f64.log10();
        let recovery: bool = self.maxrho < self.data.tau * (1. / 1.6);

        if match id {
            0 => true,
            1 => {
                self.vars.q1.cost + (10. + (self.vars.q1.level % 10) as f64).log10()
                    < self.vars.q2.cost
            }
            2 => true,
            3 => {
                self.vars.b1.cost + log10_5 < self.vars.a2.cost
                    || self.milestones[1] < 2
                    || recovery
            }
            4 => {
                self.vars.b2.cost + log10_5 < self.vars.a2.cost
                    || self.milestones[1] < 2
                    || recovery
            }
            5 => {
                self.vars.c1.cost + log10_5 < self.vars.a3.cost
                    || self.milestones[1] < 2
                    || recovery
            }
            6 => {
                self.vars.c2.cost + log10_5 < self.vars.a3.cost
                    || self.milestones[1] < 2
                    || recovery
            }
            7 => {
                self.vars.a1.cost + (4. + (self.vars.a1.level % 10) as f64 / 2.).log10()
                    < self.vars.q2.cost
                    || self.efdata.caps[2] <= self.vars.q2.level
            }
            8 | 9 => true,
            _ => false,
        } {
            BuyEval::BUY
        } else {
            BuyEval::SKIP
        }
    }

    fn get_variable_conditions(&self, id: usize) -> bool {
        match id {
            0 => self.vars.tdot.level < 4,
            1 | 2 => true,
            3 | 4 => self.milestones[0] >= 1,
            5 | 6 => self.milestones[0] >= 2,
            7 => self.milestones[1] >= 1,
            8 => self.milestones[1] >= 2,
            9 => self.milestones[1] >= 3,
            _ => false,
        }
    }

    fn tick(&mut self) {
        let logbonus = self.dt.log10() + self.multiplier;

        self.q = log10add(self.q, self.vars.q1.value + self.vars.q2.value + logbonus);
        self.tvar += self.dt * self.vars.tdot.value;

        let a = (self.vars.a1.value
            + if self.get_variable_conditions(8) {
                self.vars.a2.value
            } else {
                0.
            }
            + if self.get_variable_conditions(9) {
                self.vars.a3.value
            } else {
                0.
            })
            * (1. + 0.1 * self.milestones[2] as f64);

        if self.milestones[0] >= 1 {
            self.re = log10add(
                self.re,
                logbonus
                    + 2. * (self.vars.b1.value
                        + self.vars.b2.value
                        + self.tvar.cos().abs().log10()),
            );
        }
        if self.milestones[0] >= 2 {
            self.im = log10add(
                self.im,
                logbonus
                    + 2. * (self.vars.c1.value
                        + self.vars.c2.value
                        + self.tvar.sin().abs().log10()),
            );
        }

        match self.milestones[0] {
            0 => self.rho = log10add(self.rho, logbonus + (self.tvar.log10() + 2. * self.q) / 2.),
            1 => {
                self.rho = log10add(
                    self.rho,
                    logbonus + log10add(self.tvar.log10() + 2. * self.q, self.re * 2.) / 2.,
                )
            }
            _ => {
                self.rho = log10add(
                    self.rho,
                    logbonus
                        + a
                        + log10add(
                            self.tvar.log10() + 2. * self.q,
                            log10add(self.re * 2., self.im * 2.),
                        ) / 2.,
                )
            }
        };

        self.maxrho = self.maxrho.max(self.rho);

        self.t += self.dt / 1.5;
        self.dt *= self.ddt;
    }

    pub fn simulate(&mut self) -> SimRes {
        let mut prev_next_ms_cost: f64;

        while self.maxrho < self.goal {
            self.tick();
            prev_next_ms_cost = self.next_milestone_cost;
            if self.next_milestone_cost < 375. {
                self.update_milestones();
            }
            if self.next_milestone_cost > prev_next_ms_cost {
                println!("{} -> {}", prev_next_ms_cost, self.next_milestone_cost);
                self.efdata.caps = [u32::MAX; 10];
                //println!("{:?}", self.efdata.caps);
            }
            self.buy();
        }
        //println!("{:?}", self);
        //println!("{}", get_time_string(self.t));

        if self.t < self.best_res.t {
            SimRes {
                t: self.t,
                var_buys: Some(self.varbuys.clone()),
            }
        } else {
            SimRes {
                t: self.best_res.t,
                var_buys: match &self.best_res.var_buys {
                    None => None,
                    Some(varbuys) => Some(varbuys.clone()),
                },
            }
        }
    }

    fn get_currency(&mut self, id: usize) -> &mut f64 {
        match id {
            3 | 4 | 8 => &mut self.re,
            5 | 6 | 9 => &mut self.im,
            _ => &mut self.rho,
        }
    }

    fn buy(&mut self) {
        let mut cost: f64;
        let mut coast_eval: BuyEval;
        let mut ratio_eval: BuyEval;
        let names = ["tdot", "q1", "q2", "b1", "b2", "c1", "c2", "a1", "a2", "a3"];

        for i in (0..10).rev() {
            //if self.t2data.skip[ids[i]] { continue; }
            if self.vars.get(i).get_level() >= self.efdata.caps[i]
                || !self.get_variable_conditions(i)
            {
                continue;
            }
            cost = self.vars.get(i).get_cost();

            while *self.get_currency(i) > cost && self.get_variable_conditions(i) {
                coast_eval = self.eval_coast(i, cost);
                ratio_eval = self.eval_ratio(i);
                if coast_eval != BuyEval::SKIP {
                    if ratio_eval == BuyEval::SKIP {
                        break;
                    }
                    if coast_eval == BuyEval::FORK {
                        let mut fork: EF = self.fork();
                        let lvl: u32 = self.vars.get(i).get_level();
                        fork.efdata.caps[i] = lvl;
                        if self.depth <= 25 {
                            println!(
                                "Depth {}; Creating coasting fork for {} lvl {} -> {}, cost is {}",
                                self.depth,
                                names[i],
                                lvl,
                                lvl + 1,
                                cost
                            );
                        }
                        let res: SimRes = fork.simulate();
                        println!("Fork finished {}", res.t);
                        if res.t < self.best_res.t {
                            self.best_res.t = res.t;
                            self.best_res.var_buys = res.var_buys;
                        }
                    }
                    /*if ratio_eval == BuyEval::FORK {
                        let mut fork: T2 = self.fork();
                        fork.t2data.skip[ids[i]] = true;
                        if self.depth <= 2 {
                            println!("Depth {}; Creating ratio fork for {} lvl {}", self.depth, names[i], self.vars.get(ids[i]).get_level());
                        }
                        let res: SimRes = fork.simulate();
                        if res.t < self.best_res.t {
                            self.best_res.t = res.t;
                            self.best_res.var_buys = res.var_buys;
                        }
                    }*/

                    *self.get_currency(i) = log10sub(*self.get_currency(i), cost);
                    self.vars.getm(i).buy();
                    cost = self.vars.get(i).get_cost();

                    if self.maxrho > self.data.tau * (1. / 1.6) - 5. {
                        //println!("Buy {} lvl {} at {}; {}", names[i], self.vars.get(i).get_level(), log10tostr(cost), get_time_string(self.t));
                        self.varbuys.push(VarBuy {
                            symb: s!(names[i]),
                            lvl: self.vars.get(i).get_level(),
                            t: self.t,
                        })
                    }
                } else {
                    self.efdata.caps[i] = self.vars.get(i).get_level();
                    break;
                }
            }
        }
    }
}
