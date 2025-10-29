# Changelog for Passable Node

## v0.0.1 (2025-09-16)

### fix:
- fix occasional all-impassable issues ([e33fbfba](https://codeup.aliyun.com/deeprobotics/perception/passable_node/tree/e33fbfba843b3dea5bab05b57e5dd02cad1abbfd))
### perf:
- use queue instead of deque when BFS ([503422b8](https://codeup.aliyun.com/deeprobotics/perception/passable_node/tree/503422b868b37a6a40fd73e1893650f468813109))
- remove height ratio calculation to reduce cpu usage ([bc799ef4](https://codeup.aliyun.com/deeprobotics/perception/passable_node/tree/bc799ef462c452db5cb7777aad404787ade1773f))

## v0.0.2 (2025-10-13)

### feat:
- implement lidar coverage detection with configurable sensor poses to flag uncovered grid cells ([cd8fc4b](https://codeup.aliyun.com/deeprobotics/perception/passable_node/tree/cd8fc4b3dbcd870ff3c41875a24d9f3b4e3ade1f))
- fill lidar blind spots via a dummy elevation layer and merge the inpainted samples back into the working cloud ([5b2320a](https://codeup.aliyun.com/deeprobotics/perception/passable_node/tree/5b2320a46e7157ed78df333158b0690dd6a18711), [d50d7bf](https://codeup.aliyun.com/deeprobotics/perception/passable_node/tree/d50d7bfb8de4db80076e7a26404a80123f5fcf2c))
- evaluate elevation drops in the body frame while filtering isolated negative obstacles for more stable passability decisions ([03c55b4](https://codeup.aliyun.com/deeprobotics/perception/passable_node/tree/03c55b4316c0402a94987bb6b2c4df298d29bca5), [e148bda](https://codeup.aliyun.com/deeprobotics/perception/passable_node/tree/e148bda0047708362f135375d34cdd4587a67258))
- unify mapping and navigation flows around a single executable with launch-specific parameter sets ([fe14cad](https://codeup.aliyun.com/deeprobotics/perception/passable_node/tree/fe14cadce7368622055f54e33bf6f9c38d55ceb2))
- expose the `enable_blind_check` switch in YAML configs so blind-spot compensation can be toggled per deployment ([d3b30a9](https://codeup.aliyun.com/deeprobotics/perception/passable_node/tree/d3b30a93673acac1408b082ccb1a8daa10913ed1))

### fix:
- correct the gravity-to-body transform handling to eliminate twist errors during passability updates ([e360d82](https://codeup.aliyun.com/deeprobotics/perception/passable_node/tree/e360d82f05f751cca8ffea3c996b8412281198c9))

### refactor:
- align namespaces and filenames with the reorganized include layout ([e592d96](https://codeup.aliyun.com/deeprobotics/perception/passable_node/tree/e592d96b8cfc3d3dc6511a47e39c5295745b7b3c))

### chore:
- add shared math and utils helpers used across the passability pipeline ([3657937](https://codeup.aliyun.com/deeprobotics/perception/passable_node/tree/3657937d65c005c0fc25845d9010e15bbf0eb15f))
- prune stale configuration parameters from YAML defaults ([4b30e4b](https://codeup.aliyun.com/deeprobotics/perception/passable_node/tree/4b30e4b1d76968b0a311ad43f16c5373b963e6c2))
