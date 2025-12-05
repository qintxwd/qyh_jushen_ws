C:\Users\<你的用户名>\AppData\Local\Android\Sdk\platform-tools\adb.exe

打开手机上的 PicoVR APP（绑定头显的那个）

进入：
设备管理 → 更多设置 → 开发者模式 → 开启

2. 让 Pico 4 信任 USB 调试

用 USB-C 数据线 把 Pico 4 连接到电脑

头显内会弹出：允许 USB 调试？ → 勾选 始终允许 → 确定


adb devices

要显示有一个设备【vr】

adb install qyh_test-debug.apk