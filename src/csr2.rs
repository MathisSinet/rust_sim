use crate::utils::*;
use crate::s;



struct CSR2vars {
    q1: Variable<FirstFreeCost<ExponentialCost>, StepwiseValue>,
    q2: Variable<ExponentialCost, ExponentialValue>,
    c1: Variable<ExponentialCost, StepwiseValue>,
    n: Variable<ExponentialCost, LinearValue>,
    c2: Variable<ExponentialCost, ExponentialValue>
}

impl CSR2vars {
    fn init() -> Self {
        CSR2vars {
            q1: Variable::new(
                FirstFreeCost { model: ExponentialCost::new(10., 5.) },
                StepwiseValue::new(2., 10)
            ),
            q2: Variable::new(
                ExponentialCost::new(15., 128.),
                ExponentialValue::new(2.)
            ),
            c1: Variable::new(
                ExponentialCost::new(1e6, 16.),
                StepwiseValue::new(2., 10)
            ),
            n: Variable::new(
                ExponentialCost::new(50., 256f64.powf(3.346)),
                LinearValue::new(1.,1.)
            ),
            c2: Variable::new(
                ExponentialCost::new(1e3, 10f64.powf(5.65)),
                ExponentialValue::new(2.)
            )
        }
    }

    fn getm(&mut self, id:usize) -> &mut dyn VariableTrait {
        match id {
            0 => &mut self.q1,
            1 => &mut self.q2,
            2 => &mut self.c1,
            3 => &mut self.n,
            4 | _ => &mut self.c2
        }
    }

    fn get(&self, id:usize) -> &dyn VariableTrait {
        match id {
            0 => &self.q1,
            1 => &self.q2,
            2 => &self.c1,
            3 => &self.n,
            4 | _ => &self.c2
        }
    }

    fn set(&mut self, lvls: [u32; 5]) {
        for i in 0..5 {
            self.getm(i).set(lvls[i]);
        }
    }
}

#[derive(Clone)]
pub struct CSR2data {
    caps: [u32; 5],
    pub do_coasting: bool
}

impl Copy for CSR2data {}

pub struct CSR2state {
    pub levels: [u32; 5],
    pub q: f64
}

pub struct CSR2 {
    data: TheoryData,
    pub csr2data: CSR2data,
    pub goal: f64,
    rho: f64,
    maxrho: f64,
    multiplier: f64,
    q: f64,
    vars: CSR2vars,
    varbuys: Vec<VarBuy>,

    t: f64,
    dt: f64,
    ddt: f64,
    depth: u32,

    best_res: SimRes
}

impl CSR2 {
    pub fn new(data: TheoryData, goal: f64, state:Option<CSR2state>) -> Self {
        let mut csr2: CSR2 = CSR2 {
            data: data,
            csr2data: CSR2data { caps: [std::u32::MAX; 5], do_coasting: true },
            goal: goal,
            rho: 0.,
            maxrho: 0.,
            multiplier: 0.,
            q: 0.,
            vars: CSR2vars::init(),
            varbuys: Vec::new(),

            t: 0.,
            dt: 1.5,
            ddt: 1.0001,
            depth: 0,

            best_res: SimRes { t: std::f64::MAX, var_buys: None }
        };

        match state {
            None => (),
            Some(state) => {
                csr2.vars.set(state.levels);
                csr2.q = state.q;
            }
        }

        csr2.rho = csr2.data.rho;
        csr2.multiplier = csr2.get_multiplier(csr2.data.tau);

        csr2
    }
    
    pub fn fork(&self) -> Self {
        let mut new: CSR2 = CSR2 {
            data: self.data,
            csr2data: self.csr2data,
            goal: self.goal,
            rho: self.rho,
            maxrho: self.maxrho,
            multiplier: self.multiplier,
            q: self.q,
            vars: CSR2vars::init(),
            varbuys: self.varbuys.clone(),
            t: self.t,
            dt: self.dt,
            ddt: self.ddt,
            depth: self.depth + 1,

            best_res: SimRes { t: std::f64::MAX, var_buys: None }
        };

        for i in 0..5 {
            new.vars.getm(i).set(self.vars.get(i).get_level())
        };

        new
    }

    fn get_multiplier(&self, tau:f64) -> f64 {
        tau * 0.55075 - 200f64.log10()
    }

    fn eval_coast_one(&self, dist:f64, lbound:f64, ubound:f64) -> BuyEval {
        if dist > ubound { BuyEval::BUY } else if dist > lbound { BuyEval::FORK } else { BuyEval::SKIP }
    }

    fn eval_coast(&self, id:usize, cost:f64) -> BuyEval {
        let dist: f64 = self.goal - cost;
        if dist > 3. || !self.csr2data.do_coasting { return BuyEval::BUY; }
        match id {
            0 => self.eval_coast_one(dist, 0.65, 1.45),
            1 => self.eval_coast_one(dist, 0.15, 0.5),
            2 => self.eval_coast_one(dist, 0.85, 1.65),
            3 => self.eval_coast_one(dist, 0., 1.),
            4 => self.eval_coast_one(dist, 0., 1.),
            _ => BuyEval::SKIP
        }
    }

    fn eval_ratio(&self, id:usize) -> BuyEval {
        if match id {
            0 => {
                self.vars.q1.cost + (7. + (self.vars.q1.level % 10) as f64).log10() <
                self.vars.q2.cost.min(self.vars.n.cost).min(self.vars.c2.cost)
            },
            1 => self.vars.q2.cost + 1.8f64.log10() < self.vars.c2.cost,
            2 => {
                self.vars.c1.cost + (15. + (self.vars.c1.level % 10) as f64).log10() <
                self.vars.q2.cost.min(self.vars.n.cost).min(self.vars.c2.cost)
            }
            3 => self.vars.n.cost + 1.2f64.log10() < self.vars.c2.cost,
            4 => true,
            _ => false
        } { BuyEval::BUY } else { BuyEval::SKIP }
    }

    fn get_error(&self, n:f64) -> f64 {
        n * (8f64.sqrt() + 3.).log10() - 8f64.sqrt().log10()
    }

    fn tick(&mut self){
        let bonus: f64 = self.multiplier + self.dt.log10();

        self.q = log10add(self.q,
            self.vars.c1.value + 2. * self.vars.c2.value + self.get_error(self.vars.n.value + self.vars.c2.level as f64) + bonus
        );
        self.rho = log10add(self.rho, self.vars.q1.value * 1.15 + self.vars.q2.value + self.q + bonus);
        self.maxrho = self.maxrho.max(self.rho);

        self.t += self.dt / 1.5;
        self.dt *= self.ddt;

    }

    pub fn simulate(&mut self) -> SimRes {
        while self.maxrho < self.goal {
            self.tick();
            self.buy();
        }

        if self.t < self.best_res.t {
            SimRes { t: self.t, var_buys: Some(self.varbuys.clone()) }
        }
        else {
            SimRes { t: self.best_res.t, var_buys: Some(self.varbuys.clone()) }
        }
    }

    fn buy(&mut self) {
        let mut cost: f64;
        let mut coast_eval: BuyEval;
        let mut ratio_eval: BuyEval;
        let names = ["c2", "n", "c1", "q2", "q1"];
        let ids: [usize; 5] = [4,3,2,1,0];

        for i in 0..5 {
            //if self.t2data.skip[ids[i]] { continue; }
            if self.vars.get(ids[i]).get_level() >= self.csr2data.caps[ids[i]] { continue; }
            cost = self.vars.get(ids[i]).get_cost();
            while self.rho > cost {
                coast_eval = self.eval_coast(ids[i], cost);
                ratio_eval = self.eval_ratio(ids[i]);
                if coast_eval != BuyEval::SKIP {
                    if ratio_eval == BuyEval::SKIP {break;}
                    if coast_eval == BuyEval::FORK {
                        let mut fork: CSR2 = self.fork();
                        let lvl: u32 = self.vars.get(ids[i]).get_level();
                        fork.csr2data.caps[ids[i]] = lvl;
                        if self.depth <= 2 {
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
                    //for j in 0..7 {self.t2data.skip[j] = false;}

                    if self.maxrho > self.data.tau * 2.5 - 3. {
                        self.varbuys.push(VarBuy { symb: s!(names[i]), lvl: self.vars.get(ids[i]).get_level(), t: self.t })
                    }
                }
                else {
                    self.csr2data.caps[ids[i]] = self.vars.get(ids[i]).get_level();
                    break;
                }
            }
        }
    }
}
