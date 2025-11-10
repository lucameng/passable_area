# Changelog for Passable Area

## v0.0.1-ros2 (2025-09-16)

### fix:
- fix boundary impassable issues([8b397ad0](https://codeup.aliyun.com/deeprobotics/perception/passable_area/tree/8b397ad0402c4a48f284d346d1ce3ca1e78f1f61))
- fix impassable bug and remove height ratio calculation([01363943](https://codeup.aliyun.com/deeprobotics/perception/passable_area/tree/0136394386f8d1f5c7b2cbfd30fca955c2c21186))
### perf:
- use queue instead of deque when BFS ([1d59f301](https://codeup.aliyun.com/deeprobotics/perception/passable_area/tree/1d59f3013490058366e317d0010b0d19a66f3457))

## v0.0.2-ros2 (2025-10-13)

### feat:
- decide whether to enable blind check through the  field in the config file([9c981861](https://codeup.aliyun.com/deeprobotics/perception/passable_area/tree/9c98186194e7fe2f1b453b715275eda478ae2932))
- unify mapping and navigation program which differ from launch and config files([8c7e7d27](https://codeup.aliyun.com/deeprobotics/perception/passable_area/tree/8c7e7d27ed54d70e2241bf68d0acb10f146eb923))
- implement algorithm without consideration of nagative obstacles and lessen isolated points([afeb03e5](https://codeup.aliyun.com/deeprobotics/perception/passable_area/tree/afeb03e5c21ba7106aa0c44d8c7d4b45b09bed84))
