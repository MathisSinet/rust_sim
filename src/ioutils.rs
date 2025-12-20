use std::io::{self, Write};

fn _gets(buf: &mut String) -> io::Result<()> {
    io::stdout().flush()?;
    io::stdin().read_line(buf)?;
    *buf = buf.trim().to_string();
    Ok(())
}

pub fn input(ask: &str) -> io::Result<String> {
    let mut buffer = String::new();
    print!("{ask}");
    io::stdout().flush()?;
    io::stdin().read_line(&mut buffer)?;
    Ok(buffer.trim().to_string())
}
