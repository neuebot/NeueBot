# Missing `fri/` folder

This control architecture depends on the [KUKA Sunrise.FRI](https://www.kuka.com/en-de/products/robot-systems/software/system-software/sunriseos) package, which is sold as an extension to the standard KUKA LBR system.

For copyright reasons we cannot disclose the contents of this folder, but this README should help you configure your own system.

With the purchase of your own **KUKA Sunrise.FRI** module, you should have access to a `fri.zip` file.

You just need to unzip it in this folder in order to have the system ready to work.

```bash
$ cd neuebot_ws/src/kuka_lbr_fri
$ unzip fri.zip
```

You should end up with something like this,

```bash
kuka_lbr_fri/
├── doc
├── fri
│   ├── include
│   ├── src
│   └── CMakeLists.txt
├── include
├── src
├── CMakeLists.txt
├── Makefile
├── package.xml
└── test.ops
```