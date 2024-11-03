# launch_lifecycle_demo
https://scrapbox.io/tygoto/ROS2_Launch%E3%81%A7%E3%83%A9%E3%82%A4%E3%83%95%E3%82%B5%E3%82%A4%E3%82%AF%E3%83%AB%E5%88%B6%E5%BE%A1
上記の記事の修正版。

# 環境
Ubuntu 22.04
ROS 2 Humble

# インストール
```bash
cd ~/ros2_ws/src
git clone git@github.com:robohase/launch_lifecycle_demo.git
cd ~/ros2_ws
rosdep install -r -y -i --from-paths .
```

# ビルド
```bash
cd ~/ros2_ws/src
colcon build --symlink-install
```

# 実行
[ROS2 Launchでライフサイクル制御](https://scrapbox.io/tygoto/ROS2_Launch%E3%81%A7%E3%83%A9%E3%82%A4%E3%83%95%E3%82%B5%E3%82%A4%E3%82%AF%E3%83%AB%E5%88%B6%E5%BE%A1)
この記事を参考にする