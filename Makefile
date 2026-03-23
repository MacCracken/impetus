.PHONY: check fmt clippy test audit deny vet semver bench coverage build doc clean all

# Run all CI checks locally
check: fmt clippy test audit

# Full check including supply-chain
all: check deny vet semver doc

# Format check
fmt:
	cargo fmt --all -- --check

# Lint (zero warnings)
clippy:
	cargo clippy --all-features --all-targets -- -D warnings

# Run test suite
test:
	cargo test --all-features

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

# Generate coverage report
coverage:
	cargo llvm-cov --all-features --html --output-dir coverage/
	@echo "Coverage report: coverage/html/index.html"

# Build release
build:
	cargo build --release --all-features

# Generate documentation
doc:
	RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features

# Clean build artifacts
clean:
	cargo clean
	rm -rf coverage/
