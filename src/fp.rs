use crate::s;
use crate::utils::*;

pub fn stepwise_sum(mut level: u32, base: u32, length: u32) -> u32 {
    if level <= length {
        return level;
    }
    level -= length;
    let cycles = level / length;
    let modpart = level - cycles * length;
    (base as f64 * (cycles + 1) as f64 * ((length * cycles) as f64 / 2. + modpart as f64))
        as u32
        + length
        + level
}

fn u32_is_powerof2(mut n: u32) -> bool {
    if n == 0 {
        return false;
    }
    while n != 1 {
        if n & 1 == 1 {
            return false;
        }
        n >>= 1;
    }
    true
}

fn u32_log2(mut n: u32) -> u32 {
    if n == 0 {
        return u32::MAX;
    }
    let mut res: u32 = 0;
    while n != 1 {
        n >>= 1;
        res += 1;
    }
    res
}

#[derive(Debug)]
struct Q1Value {
    stepwise_part: StepwiseValue,
}

impl Value for Q1Value {
    fn recompute(&self, level: u32) -> f64 {
        let stepwise_value = self.stepwise_part.recompute(level);
        //stepwise_value - log10add(0., 1000. / (level as f64).powf(1.5))
        stepwise_value - (1. + 1000. / (level as f64).powf(1.5)).log10()
    }
}

#[derive(Debug)]
struct R1Value {
    stepwise_part: StepwiseValue,
}

impl Value for R1Value {
    fn recompute(&self, level: u32) -> f64 {
        let stepwise_value = self.stepwise_part.recompute(level);
        //stepwise_value - log10add(0., 1e9 / (level as f64).powi(4))
        stepwise_value - (1. + 1e9 / (level as f64).powi(4)).log10()
    }
}

#[derive(Debug)]
struct SValue {}

impl Value for SValue {
    fn recompute(&self, level: u32) -> f64 {
        let cutoffs = [32, 39];
        if level < cutoffs[0] {
            1. + level as f64 * 0.15
        } else if level < cutoffs[1] {
            self.recompute(cutoffs[0] - 1) + 0.15 + (level - cutoffs[0]) as f64 * 0.2
        } else {
            self.recompute(cutoffs[1] - 1) + 0.2 + (level - cutoffs[1]) as f64 * 0.15
        }
    }
}

#[derive(Debug)]
struct FPvars {
    c1: Variable<FirstFreeCost<ExponentialCost>, StepwiseValue>,
    c2: Variable<CompositeCost<ExponentialCost, ExponentialCost>, ExponentialValue>,
    q1: Variable<FirstFreeCost<ExponentialCost>, Q1Value>,
    q2: Variable<ExponentialCost, EmptyValue>,
    r1: Variable<FirstFreeCost<CompositeCost<ExponentialCost, ExponentialCost>>, R1Value>,
    n: Variable<ExponentialCost, EmptyValue>,
    s: Variable<ExponentialCost, SValue>,
}

impl FPvars {
    fn init() -> Self {
        FPvars {
            c1: Variable::new(
                FirstFreeCost {
                    model: ExponentialCost::new(10., 1.4),
                },
                StepwiseValue::new(150., 100),
            ),
            c2: Variable::new(
                CompositeCost {
                    model1: ExponentialCost::new(1e15, 40.),
                    model2: ExponentialCost::new(1e37, 16.42),
                    cutoff: 15,
                },
                ExponentialValue::new(2.),
            ),
            q1: Variable::new(
                FirstFreeCost {
                    model: ExponentialCost::new(1e35, 12.),
                },
                Q1Value {
                    stepwise_part: StepwiseValue::new(10., 10),
                },
            ),
            q2: Variable::new(ExponentialCost::new(1e76, 1e3), EmptyValue {}),
            r1: Variable::new(
                FirstFreeCost {
                    model: CompositeCost {
                        model1: ExponentialCost::new(1e80, 25.),
                        model2: ExponentialCost::new_fullbase(480., 150.),
                        cutoff: 285,
                    },
                },
                R1Value {
                    stepwise_part: StepwiseValue::new(2., 5),
                },
            ),
            n: Variable::new(ExponentialCost::new(1e4, 3e6), EmptyValue {}),
            s: Variable::new(ExponentialCost::new_fullbase(730., 1e30), SValue {}),
        }
    }

    fn getm(&mut self, id: usize) -> &mut dyn VariableTrait {
        match id {
            0 => &mut self.c1,
            1 => &mut self.c2,
            2 => &mut self.q1,
            3 => &mut self.q2,
            4 => &mut self.r1,
            5 => &mut self.n,
            _ => &mut self.s,
        }
    }

    fn get(&self, id: usize) -> &dyn VariableTrait {
        match id {
            0 => &self.c1,
            1 => &self.c2,
            2 => &self.q1,
            3 => &self.q2,
            4 => &self.r1,
            5 => &self.n,
            _ => &self.s,
        }
    }

    fn set(&mut self, lvls: [u32; 7]) {
        for i in 0..7 {
            self.getm(i).set(lvls[i]);
        }
    }
}

#[derive(Debug, Clone)]
pub struct FPdata {
    caps: [u32; 7],
    pub do_coasting: bool,
}

impl Copy for FPdata {}

pub struct FPstate {
    pub levels: [u32; 7],
    pub t: f64,
    pub q: f64,
    pub r: f64,
}

#[derive(Debug, Clone, Copy)]
struct FPcache {
    n: u32,
    tn: f64,
    un: f64,
    sn: f64,
}

#[derive(Debug)]
pub struct FP {
    data: TheoryData,
    pub fpdata: FPdata,
    pub goal: f64,
    rho: f64,
    maxrho: f64,
    multiplier: f64,

    tvar: f64,
    q: f64,
    r: f64,
    cache: FPcache,
    update_cache: bool,
    vars: FPvars,
    varbuys: Vec<VarBuy>,

    rmilestone: bool,

    t: f64,
    dt: f64,
    ddt: f64,
    depth: u32,

    best_res: SimRes,
}

impl FP {
    pub fn new(data: TheoryData, goal: f64, state: Option<FPstate>) -> Self {
        let mut sim: FP = FP {
            data: data,
            fpdata: FPdata {
                caps: [u32::MAX; 7],
                do_coasting: true,
            },
            goal: goal,
            rho: 0.,
            maxrho: 0.,
            multiplier: 0.,

            tvar: 0.,
            q: 0.,
            r: 0.,
            cache: FPcache {
                n: 1,
                tn: 0.,
                un: 0.,
                sn: 0.,
            },
            update_cache: true,
            vars: FPvars::init(),
            varbuys: Vec::new(),
            rmilestone: false,

            t: 0.,
            dt: 1.5,
            ddt: 1.0001,
            depth: 0,

            best_res: SimRes::default(),
        };

        match state {
            None => (),
            Some(state) => {
                sim.vars.set(state.levels);
                sim.tvar = state.t;
                sim.q = state.q;
                sim.r = state.r
            }
        }

        sim.rho = sim.data.rho;
        sim.multiplier = sim.get_multiplier(sim.data.tau);

        sim
    }

    pub fn fork(&self) -> Self {
        let mut new: FP = FP {
            data: self.data,
            fpdata: self.fpdata,
            goal: self.goal,
            rho: self.rho,
            maxrho: self.maxrho,
            multiplier: self.multiplier,

            tvar: self.tvar,
            q: self.q,
            r: self.r,
            cache: self.cache,
            update_cache: self.update_cache,
            vars: FPvars::init(),
            varbuys: self.varbuys.clone(),
            rmilestone: self.rmilestone,

            t: self.t,
            dt: self.dt,
            ddt: self.ddt,
            depth: self.depth + 1,

            best_res: SimRes::default(),
        };

        for i in 0..7 {
            new.vars.getm(i).set(self.vars.get(i).get_level())
        }

        new
    }

    fn get_multiplier(&self, tau: f64) -> f64 {
        tau * 0.331 + 5f64.log10()
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
        //return BuyEval::BUY;
        let dist: f64 = self.goal - cost;
        //return BuyEval::BUY;
        if dist > 3. || !self.fpdata.do_coasting {
            return BuyEval::BUY;
        }
        match id {
            0 => self.eval_coast_one(dist, 0.3, 1.5),
            1 => self.eval_coast_one(dist, 0.15, 0.5),
            2 => self.eval_coast_one(dist, 0.3, 1.5),
            3 => self.eval_coast_one(dist, 0.3, 1.),
            4 => self.eval_coast_one(dist, 0.1, 1.5),
            5 => self.eval_coast_one(dist, 0., 1.5),
            6 => BuyEval::BUY,
            _ => BuyEval::SKIP,
        }
    }

    fn eval_ratio(&self, id: usize) -> BuyEval {
        //return BuyEval::BUY;
        if match id {
            0 => {
                let mod100 = self.vars.c1.level % 100;
                let remaining_levels = 101 - mod100;
                //const P = 1.4;
                let remaining_cost =
                    self.vars.c1.cost + ((1.4f64.powi(remaining_levels as i32) - 1.) / 0.4).log10();
                (mod100 > 85
                    && remaining_cost < self.vars.c2.cost + 0.1
                    && remaining_cost < self.vars.s.cost)
                    || (self.vars.c1.cost + (mod100 as f64 + 1.).log10()
                        < self.vars.c2.cost.min(self.vars.s.cost))
            }
            1 => self.vars.c2.cost + 0.1 < self.vars.s.cost,
            2 => {
                self.vars.q1.cost + 1.5 * ((self.vars.q1.level % 10) as f64 + 1.).log10()
                    < self.vars.q2.cost
            }
            3 => self.vars.q2.cost + 0.1 < self.vars.s.cost,
            4 => true,
            5 => true,
            6 => true,
            _ => false,
        } {
            BuyEval::BUY
        } else {
            BuyEval::SKIP
        }
    }

    fn get_t(&self, n: u32) -> f64 {
        if n == 0 {
            return 0.;
        }
        let log2n = u32_log2(n);
        if u32_is_powerof2(n) {
            (1. + 2f64.powf(2. * log2n as f64 + 1.)) / 3.
        } else {
            let i = n - (1 << log2n);
            self.get_t(1 << log2n) + 2. * self.get_t(i) + self.get_t(i + 1) - 1.
        }
    }

    fn get_v(&self, n: u32) -> f64 {
        if n == 0 {
            return 0.;
        }
        let log2n = u32_log2(n);
        if u32_is_powerof2(n) {
            2f64.powf(2. * log2n as f64)
        } else {
            let i = n - (1 << log2n);
            2f64.powf(2. * log2n as f64) + 3. * self.get_v(i)
        }
    }

    fn get_u(&self, n: u32) -> f64 {
        (4. / 3.) * self.get_v(n) - (1. / 3.)
    }

    fn get_s(&self, n: u32) -> f64 {
        (1f64 / 3f64).log10() + log10sub(2f64.log10() + 3f64.log10() * n as f64, 3f64.log10())
    }

    fn approx(&self, n: u32) -> f64 {
        (1f64 / 6f64).log10() + log10add(2f64.log10() * 2. * (n + 1) as f64, 2f64.log10())
    }

    fn tick(&mut self) {
        if self.update_cache {
            self.cache.n = (1
                + stepwise_sum(self.vars.n.level, 1, 40)
                + stepwise_sum((self.vars.n.level as i32 - 30).max(0) as u32, 1, 35) * 2
                + (stepwise_sum((self.vars.n.level as i32 - 69).max(0) as u32, 1, 30) as f64 * 2.4
                    + 0.001)
                    .floor() as u32)
                .min(20000);
            self.cache.tn = self.get_t(self.cache.n);
            self.cache.un = self.get_u(self.cache.n);
            self.cache.sn = self.get_s((self.cache.n as f64 + 0.001).sqrt().floor() as u32);
            self.update_cache = false;
        }

        self.tvar += self.dt;
        self.q = log10add(
            self.q,
            self.vars.q1.value
                + self.approx(self.vars.q2.level)
                + self.cache.un.log10() * (7. + self.vars.s.value)
                - 3.
                + self.dt.log10(),
        );
        self.r = log10add(
            self.r,
            self.vars.r1.value
                + (self.cache.tn.log10() + self.cache.un.log10())
                    * (if self.rmilestone {
                        (self.cache.un * 2.).log10() / 2.
                    } else {
                        (self.cache.n as f64).log10()
                    })
                + self.cache.sn * 2.8
                + self.dt.log10(),
        );
        self.rho = log10add(
            self.rho,
            self.multiplier
                + self.vars.c1.value
                + self.vars.c2.value
                + self.cache.tn.log10() * (5. + self.vars.s.value)
                + self.tvar.log10()
                + self.dt.log10()
                + self.q
                + self.r,
        );

        self.maxrho = self.maxrho.max(self.rho);

        self.t += self.dt / 1.5;
        self.dt *= self.ddt;
    }

    pub fn simulate(&mut self) -> SimRes {
        while self.maxrho < self.goal {
            if self.rho.max(self.data.tau * (1. / 0.3)) >= 1500. {
                self.rmilestone = true;
            }
            self.tick();
            self.buy();
        }
        //println!("{:?}", self);

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

    fn buy(&mut self) {
        let mut cost: f64;
        let mut coast_eval: BuyEval;
        let mut ratio_eval: BuyEval;
        let names = ["s", "n", "r1", "q2", "q1", "c2", "c1"];
        let ids: [usize; 7] = [6, 5, 4, 3, 2, 1, 0];

        for i in 0..7 {
            //if self.t2data.skip[ids[i]] { continue; }
            if self.vars.get(ids[i]).get_level() >= self.fpdata.caps[ids[i]] {
                continue;
            }
            cost = self.vars.get(ids[i]).get_cost();
            while self.rho > cost {
                coast_eval = self.eval_coast(ids[i], cost);
                ratio_eval = self.eval_ratio(ids[i]);
                if coast_eval != BuyEval::SKIP {
                    if ratio_eval == BuyEval::SKIP {
                        break;
                    }
                    if coast_eval == BuyEval::FORK {
                        let mut fork: FP = self.fork();
                        let lvl: u32 = self.vars.get(ids[i]).get_level();
                        fork.fpdata.caps[ids[i]] = lvl;
                        if self.depth <= 10 {
                            //println!("Depth {}; Creating coasting fork for {} lvl {}", self.depth, names[i], lvl);
                        }
                        let res: SimRes = fork.simulate();
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

                    self.rho = log10sub(self.rho, cost);
                    self.vars.getm(ids[i]).buy();
                    cost = self.vars.get(ids[i]).get_cost();
                    if i == 1 {
                        self.update_cache = true;
                    }
                    //for j in 0..7 {self.t2data.skip[j] = false;}

                    if self.maxrho > self.data.tau * (1. / 0.3) - 5. {
                        self.varbuys.push(VarBuy {
                            symb: s!(names[i]),
                            lvl: self.vars.get(ids[i]).get_level(),
                            t: self.t,
                        })
                    }
                } else {
                    self.fpdata.caps[ids[i]] = self.vars.get(ids[i]).get_level();
                    break;
                }
            }
        }
    }
}
