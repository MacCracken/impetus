.PHONY: check fmt clippy test audit deny vet semver bench coverage build doc clean all

# Run all CI checks locally
check: fmt clippy test audit

# Full check including supply-chain
all: check deny vet semver doc

# Format check
fmt:
	cargo fmt --all -- --check

# Lint (zero warnings) — every backend combination (3d wins when both are on)
clippy:
	cargo clippy --all-features --all-targets -- -D warnings
	cargo clippy --all-targets -- -D warnings
	cargo clippy --no-default-features --features 2d,serialize --all-targets -- -D warnings
	cargo clippy --no-default-features --features 3d --all-targets -- -D warnings
	cargo clippy --no-default-features --all-targets -- -D warnings

# Run test suite — 2D backend (default) and 3D backend (--all-features)
test:
	cargo test --features serialize
	cargo test --all-features
	cargo test --no-default-features

# Security audit
audit:
	cargo audit

# Supply-chain checks (cargo-deny)
deny:
	cargo deny check

# Supply-chain verification (cargo-vet)
vet:
	cargo vet --locked

# SemVer compatibility check
semver:
	cargo semver-checks check-release

# Run benchmarks with history
bench:
	./scripts/bench-history.sh

# Generate coverage report (2D and 3D runs merged)
coverage:
	cargo llvm-cov clean --workspace
	cargo llvm-cov --no-report --features serialize
	cargo llvm-cov --no-report --all-features
	cargo llvm-cov report --html --output-dir coverage/
	@echo "Coverage report: coverage/html/index.html"

# Build release
build:
	cargo build --release --all-features

# Generate documentation
doc:
	RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features
	RUSTDOCFLAGS="-D warnings" cargo doc --no-deps

# Clean build artifacts
clean:
	cargo clean
	rm -rf coverage/
