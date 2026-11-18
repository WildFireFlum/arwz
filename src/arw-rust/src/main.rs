use arw_archive::{
    archive::{pack_with_level as pack, unpack_with_level as unpack},
    log_to, Level,
};
use std::{
    env, fs, io,
    path::{Path, PathBuf},
    time::Instant,
};

/// Depth-first iterator that keeps open directory handles instead of
/// collecting all paths in a tree before processing starts.
struct FileWalker {
    directories: Vec<fs::ReadDir>,
}

impl FileWalker {
    fn new(root: &Path) -> Result<Self, String> {
        let first = fs::read_dir(root).map_err(|e| format!("{}: {e}", root.display()))?;
        Ok(Self {
            directories: vec![first],
        })
    }
}

impl Iterator for FileWalker {
    type Item = Result<PathBuf, String>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let entry = match self.directories.last_mut() {
                Some(directory) => match directory.next() {
                    Some(Ok(entry)) => entry,
                    Some(Err(error)) => return Some(Err(error.to_string())),
                    None => {
                        self.directories.pop();
                        continue;
                    }
                },
                None => return None,
            };
            let path = entry.path();
            let kind = match entry.file_type() {
                Ok(kind) => kind,
                Err(error) => return Some(Err(format!("{}: {error}", path.display()))),
            };
            if kind.is_symlink() {
                return Some(Err(format!("symlinks are unsupported: {}", path.display())));
            }
            if kind.is_dir() {
                match fs::read_dir(&path) {
                    Ok(directory) => self.directories.push(directory),
                    Err(error) => {
                        return Some(Err(format!("{}: {error}", path.display())));
                    }
                }
            } else if kind.is_file() {
                return Some(Ok(path));
            }
        }
    }
}
/// Process matching files recursively while preserving their relative paths.
///
/// Input files are read and handled one at a time. The source and destination
/// trees may not overlap, existing outputs are not overwritten, and
/// `verify` compares restored bytes without writing them.
fn process_tree(cmd: &str, src: &Path, dst: &Path, level: Level) -> Result<(), String> {
    if !src.is_dir() {
        return Err(format!("not a directory: {}", src.display()));
    }
    if src.canonicalize().ok() == dst.canonicalize().ok()
        || dst.starts_with(src)
        || src.starts_with(dst)
    {
        return Err("source and destination directories must not overlap".into());
    }
    let suffix = if cmd == "compress" || cmd == "verify" {
        ".arw"
    } else {
        ".aaa"
    };
    let started = Instant::now();
    let mut ok = 0usize;
    let mut failed = 0usize;
    let mut processed = 0usize;
    let mut in_bytes = 0u64;
    let mut out_bytes = 0u64;
    for path in FileWalker::new(src)? {
        let path = path?;
        if !path
            .file_name()
            .unwrap_or_default()
            .to_string_lossy()
            .to_ascii_lowercase()
            .ends_with(suffix)
        {
            continue;
        }
        processed += 1;
        let rel = path.strip_prefix(src).map_err(|e| e.to_string())?;
        let target_rel = if cmd == "compress" || cmd == "verify" {
            PathBuf::from(format!("{}.aaa", rel.display()))
        } else {
            let s = rel.to_string_lossy();
            PathBuf::from(&s[..s.len() - 4])
        };
        let target = dst.join(target_rel);
        let timer = Instant::now();
        let data = fs::read(&path).map_err(|e| e.to_string())?;
        in_bytes += data.len() as u64;
        log_to(
            &mut io::stdout(),
            level,
            Level::Info,
            &format!("[{cmd} {processed}] START {}", rel.display()),
        )
        .map_err(|e| e.to_string())?;
        let result = if cmd == "compress" {
            pack(&data, level)
        } else if cmd == "decompress" {
            unpack(&data, level)
        } else {
            let archive = fs::read(&target).map_err(|e| format!("{}: {e}", target.display()))?;
            unpack(&archive, level).and_then(|restored| {
                if restored == data {
                    Ok(Vec::new())
                } else {
                    Err("byte comparison failed".into())
                }
            })
        };
        match result {
            Ok(bytes) => {
                if cmd != "verify" {
                    if target.exists() {
                        return Err(format!("refusing to overwrite {}", target.display()));
                    }
                    if let Some(parent) = target.parent() {
                        fs::create_dir_all(parent).map_err(|e| e.to_string())?
                    }
                    fs::write(&target, &bytes).map_err(|e| e.to_string())?;
                    out_bytes += bytes.len() as u64;
                }
                ok += 1;
                log_to(
                    &mut io::stdout(),
                    level,
                    Level::Info,
                    &format!(
                        "[{cmd} {processed}] OK {} ({:.3}s)",
                        rel.display(),
                        timer.elapsed().as_secs_f64()
                    ),
                )
                .map_err(|e| e.to_string())?;
            }
            Err(e) => {
                failed += 1;
                println!("[{cmd}] FAILED {}: {e}", rel.display());
            }
        }
    }
    if processed == 0 {
        return Err(format!("no {suffix} files found"));
    }
    println!("{cmd} summary: {ok}/{processed} succeeded, {failed} failed; {in_bytes} input bytes, {out_bytes} output bytes; {:.3}s",started.elapsed().as_secs_f64());
    if failed > 0 {
        return Err("one or more files failed".into());
    }
    Ok(())
}
/// Parse the command line and dispatch single-file or directory operations.
fn run() -> Result<(), String> {
    let mut args = env::args().skip(1);
    let cmd=args.next().ok_or("usage: arw-archive <compress|decompress|verify|pack|unpack> [--log-level quiet|info|debug] INPUT OUTPUT")?;
    let mut level = Level::Info;
    let mut positional = Vec::new();
    while let Some(a) = args.next() {
        if a == "--log-level" {
            level = Level::parse(&args.next().ok_or("missing log level")?)
                .ok_or("invalid log level")?;
        } else if a == "--help" || a == "-h" {
            println!("arw-archive <compress|decompress|verify|pack|unpack> [--log-level quiet|info|debug] INPUT OUTPUT");
            return Ok(());
        } else {
            positional.push(a)
        }
    }
    if positional.len() != 2 {
        return Err("expected input and output paths".into());
    }
    let input = Path::new(&positional[0]);
    let output = Path::new(&positional[1]);
    if cmd == "compress" || cmd == "decompress" || cmd == "verify" {
        return process_tree(&cmd, input, output, level);
    }
    if cmd != "verify" && output.exists() {
        return Err(format!("refusing to overwrite {}", output.display()));
    }
    let data = fs::read(input).map_err(|e| e.to_string())?;
    match cmd.as_str() {
        "pack" => {
            let archive = pack(&data, level)?;
            fs::write(output, archive).map_err(|e| e.to_string())?;
        }
        "unpack" => {
            let raw = unpack(&data, level)?;
            fs::write(output, raw).map_err(|e| e.to_string())?;
        }
        "verify" => {
            let raw = unpack(&data, level)?;
            if raw != fs::read(output).map_err(|e| e.to_string())? {
                return Err("byte comparison failed".into());
            }
            println!("BIT-IDENTICAL: {}", output.display());
        }
        _ => return Err("command must be compress, decompress, verify, pack, or unpack".into()),
    }
    Ok(())
}
fn main() {
    if let Err(e) = run() {
        eprintln!("error: {e}");
        std::process::exit(2)
    }
}

#[cfg(test)]
mod tests {
    use super::FileWalker;
    use std::{collections::HashSet, fs, time::SystemTime};

    #[test]
    fn file_walker_yields_nested_files() {
        let stamp = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let root = std::env::temp_dir().join(format!(
            "arw-archive-walk-test-{}-{stamp}",
            std::process::id()
        ));
        fs::create_dir_all(root.join("nested/deeper")).unwrap();
        fs::write(root.join("one.ARW"), b"one").unwrap();
        fs::write(root.join("nested/deeper/two.ARW"), b"two").unwrap();

        let found: HashSet<_> = FileWalker::new(&root)
            .unwrap()
            .map(|path| path.unwrap().strip_prefix(&root).unwrap().to_path_buf())
            .collect();
        assert_eq!(
            found,
            [
                std::path::PathBuf::from("one.ARW"),
                std::path::PathBuf::from("nested/deeper/two.ARW"),
            ]
            .into_iter()
            .collect()
        );
        fs::remove_dir_all(root).unwrap();
    }
}
