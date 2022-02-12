## Releasing guide

Create a new branch

```
git fetch upstream
git checkout upstream/master
git checkout -b v0.2.0
```

* Update version in README
* Update version in Cargo.toml
* Update CHANGELOG
* Update index in top-level documentation (lib.rs)
* Check that all feature flags mentioned there also appear under
  `[package.metadata.docs.rs]` in Cargo.toml

```
git commit -am 'v0.2.0'
```

Push the new branch

```
git push --set-upstream upstream v0.2.0
```

Create a PR and check CI passes.

Merge into master, you can do this from the command line:

```
git push --set-upstream upstream v0.2.0:master
```

Create and push a tag

```
git tag -a 'v0.2.0' -m 'v0.2.0'
git push upstream refs/tags/v0.2.0
```

Checkout in a clean tree and publish

```
cd [clean tree]
git pull upstream
cargo publish --features stm32h743
```
