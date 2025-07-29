use std::fmt::Debug;

#[macro_export]
macro_rules! s {
    ($x:expr) => {
        String::from($x)
    };
}

pub fn log10tostr (x:f64) -> String {
    let m = 10f64.powf(x - x.floor());

    s!(format!("{}e{}", (100. * m).round() / 100., x.floor()))
}

pub fn strtolog10(s: String) -> f64 {
    let split: Vec<&str> = s.split("e").collect();
    let m: f64 = split[0].parse().unwrap();
    let e: f64 = split[1].parse().unwrap();

    e + m.log10()
}

pub fn log10add(a:f64, b:f64) -> f64 {
    let max:f64 = a.max(b);
    let min:f64 = a.min(b);
    let whole1:f64 = max.floor();
    let frac1:f64 = 10f64.powf(max - whole1);
    let whole2:f64 = min.floor();
    let frac2:f64 = 10f64.powf(min - whole2);

    if whole1 > whole2 + 40. {return max};

    whole1 + (frac1 + frac2 / 10f64.powf(whole1 - whole2)).log10()
}

/*pub fn log10add(a:f64, b:f64) -> f64 {
    let max:f64 = a.max(b);
    let min:f64 = a.min(b);

    if max > min + 40. {return max};

    max + (1. + 10f64.powf(min - max)).log10()
}*/

pub fn log10sub(a:f64, b:f64) -> f64 {
    let max:f64 = a.max(b);
    let min:f64 = a.min(b);
    let whole1:f64 = max.floor();
    let frac1:f64 = 10f64.powf(max - whole1);
    let whole2:f64 = min.floor();
    let frac2:f64 = 10f64.powf(min - whole2);

    if whole1 > whole2 + 40. {return max};

    whole1 + (frac1 - frac2 / 10f64.powf(whole1 - whole2)).log10()
}

pub fn get_time_string(time:f64) -> String {
    let mut mins:f64 = (time / 60.).floor();
    let mut hours:f64 = (mins / 60.).floor();
    mins -= 60. * hours;
    let days = (hours / 24f64).floor();
    hours -= 24. * days;

    format!("{days}d {hours}h {mins}min")
}

// Costs

pub trait Cost {
    fn get_cost(&self, level: u32) -> f64;
}

pub struct ExponentialCost {
    base: f64,
    exp: f64
}

impl ExponentialCost {
    pub fn new(base: f64, exp: f64) -> Self {
        ExponentialCost {
            base: base.log10(),
            exp: exp.log10(),
        }
    }
}

impl Cost for ExponentialCost {
    fn get_cost(&self, level: u32) -> f64 {
        self.base + self.exp * level as f64
    }
}

pub struct FirstFreeCost<T: Cost> {
    pub model: T
}

impl<T: Cost> Cost for FirstFreeCost<T> {
    fn get_cost(&self, level: u32) -> f64 {
        self.model.get_cost(level - 1)
    }
}

// Value

pub trait Value {
    fn recompute(&self, level: u32) -> f64;
}

pub struct StepwiseValue {
    exp: f64,
    len: u32
}

impl StepwiseValue {
    pub fn new(exp: f64, len: u32) -> Self {
        StepwiseValue { exp: exp, len: len }
    }
}

impl Value for StepwiseValue {
    fn recompute(&self, level: u32) -> f64 {
        let intpart: f64 = (level / self.len) as f64;
        let modpart: f64 = (level as f64) - intpart * (self.len as f64);
        let d: f64 = (self.len as f64) / (self.exp - 1.);

        log10sub((d + modpart).log10() + self.exp.log10() * intpart, d.log10())
    }
}

pub struct ExponentialValue {
    base: f64
}

impl ExponentialValue {
    pub fn new(base: f64) -> Self {
        ExponentialValue { base: base.log10() }
    }
}

impl Value for ExponentialValue {
    fn recompute(&self, level: u32) -> f64 {
        self.base * level as f64
    }
}

pub struct LinearValue {
    base: f64,
    offset: f64
}

impl LinearValue {
    pub fn new(base: f64, offset: f64) -> Self {
        LinearValue { base: base, offset: offset }
    }
}

impl Value for LinearValue {
    fn recompute(&self, level: u32) -> f64 {
        self.base * level as f64 + self.offset
    }
}

// Variable

pub trait VariableTrait {
    fn get_level(&self) -> u32;
    fn get_value(&self) -> f64;
    fn get_cost(&self) -> f64;
    fn buy(&mut self);
    fn set(&mut self, level: u32);
}

pub struct Variable<T: Cost, U: Value> {
    costmodel: T,
    valuemodel: U,
    pub level: u32,
    pub value: f64,
    pub cost: f64
}

impl<T: Cost, U: Value> Variable<T, U> {
    pub fn new(costmodel: T, valuemodel: U) -> Self {
        let mut var:Variable<T,U> = Variable {
            costmodel: costmodel,
            valuemodel: valuemodel,
            level: 1,
            value: 1.,
            cost: 1.
        };

        var.recompute();
        var
    }

    fn _get_cost(&self) -> f64 {
        self.costmodel.get_cost(self.level)
    }

    fn recompute(&mut self) {
        self.value = self.valuemodel.recompute(self.level);
        self.cost = self.costmodel.get_cost(self.level);
    }

    pub fn buy(&mut self) {
        self.level += 1;
        self.recompute();
    }

    pub fn set(&mut self, level: u32) {
        self.level = level;
        self.recompute();
    }
}

impl<T: Cost, U:Value> VariableTrait for Variable<T, U> {
    fn get_level(&self) -> u32 {
        self.level
    }

    fn get_value(&self) -> f64 {
        self.value
    }

    fn get_cost(&self) -> f64 {
        self.cost
    }

    fn buy(&mut self) {
        self.buy();
    }

    fn set(&mut self, level: u32) {
        self.set(level);
    }
}

#[derive(Clone)]
pub struct TheoryData {
    pub tau: f64,
    pub students: u32,
    pub rho: f64
}

impl Copy for TheoryData{}

pub struct VarBuy {
    pub symb: String,
    pub lvl: u32,
    pub t: f64
}

impl Debug for VarBuy {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: lvl {}, {}", self.symb, self.lvl, get_time_string(self.t))
    }
}

impl Clone for VarBuy {
    fn clone(&self) -> Self {
        VarBuy { symb: self.symb.clone(), lvl: self.lvl, t: self.t }
    }
}

#[derive(PartialEq, Debug)]
pub enum BuyEval {
    BUY, FORK, SKIP
}

pub struct SimRes {
    pub t: f64,
    pub var_buys: Option<Vec<VarBuy>>
}