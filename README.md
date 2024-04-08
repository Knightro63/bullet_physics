# bullet\_physics

[![Pub Version](https://img.shields.io/pub/v/bullet_physics)](https://pub.dev/packages/bullet_physics)
[![analysis](https://github.com/Knightro63/bullet_physics/actions/workflows/flutter.yml/badge.svg)](https://github.com/Knightro63/bullet_physics/actions/)
[![License: BSD](https://img.shields.io/badge/license-BSD-purple.svg)](https://opensource.org/licenses/BSD)

A 3D physics engine for dart (based on [bullet3](https://github.com/bulletphysics/bullet3) physics ) that allows users to add physics support to their 3D projects.

This is a dart conversion of [jbullet](https://github.com/bubblecloud/jbullet) which is a conversion of [bullet3](https://github.com/bulletphysics/bullet3), originally created by erwincoumans [@erwincoumans](https://github.com/erwincoumans).

## Usage

This project is a basic physics engine for 3D modeling. This package includes RigidBodies, and Joints.

### Getting started

To get started add bullet_physics, and your favorite 3D rendering engine to your pubspec.yaml file.

The bullet World is the main scene that has all of the objects that will be manipulated to the scene. To get started add the bullet world then all of the objects in it.

If there is no shapes or type in the ObjectConfigure class it will not work. If you need a RigidBody use shapes. If you need a Joint use type.


## Example


## Contributing

Contributions are welcome.
In case of any problems look at [existing issues](https://github.com/Knightro63/bullet_physics/issues), if you cannot find anything related to your problem then open an issue.
Create an issue before opening a [pull request](https://github.com/Knightro63/bullet_physics/pulls) for non trivial fixes.
In case of trivial fixes open a [pull request](https://github.com/Knightro63/bullet_physics/pulls) directly.

## Additional Information

This plugin is only for performing basic physics. While this can be used as a standalone project it does not render scenes.
