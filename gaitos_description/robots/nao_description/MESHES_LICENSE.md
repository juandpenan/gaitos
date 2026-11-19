# NAO meshes: attribution

The meshes and the texture of the NAO (`meshes/V40/*.dae`, `materials/texture/textureNAO.png`)
come from the Gazebo Fuel model
[NAO with Ignition position controller](https://app.gazebosim.org/OpenRobotics/fuel/models/NAO%20with%20Ignition%20position%20controller)
(OpenRobotics, Carlos Agüero; taken from [osrf/robocup3ds](https://github.com/osrf/robocup3ds)),
licensed under [Creative Commons Attribution 4.0](https://creativecommons.org/licenses/by/4.0/)
(CC BY 4.0). They are not part of the gaitos sources: gaitos_description downloads them at
build time (version 2 of the model).

Changes: the `.dae` are installed as they are; `mjcf/meshes/*.obj` are converted from them by
`scripts/dae_to_obj.py` (same geometry, normals and texture coordinates) for the MuJoCo model.
