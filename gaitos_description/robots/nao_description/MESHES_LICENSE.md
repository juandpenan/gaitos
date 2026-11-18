# NAO meshes: license

The meshes and textures of the NAO in this folder (`meshes/`, `texture/`) are made by SoftBank
Robotics (Aldebaran) and distributed by ros-naoqi (`nao_meshes`, installer
`naomeshes-0.6.7-linux-x64-installer.run`). They are **not** covered by the license of gaitos
(Apache 2.0).

They are licensed under the **Creative Commons Attribution-NonCommercial-NoDerivatives 4.0
International Public License** (CC BY-NC-ND 4.0):
https://creativecommons.org/licenses/by-nc-nd/4.0/legalcode

- Attribution: SoftBank Robotics, NAO meshes.
- NonCommercial: no commercial use.
- NoDerivatives: do not distribute modified versions.

They are downloaded only when gaitos_description is built with `-DGAITOS_NAO_MESHES=ON`, which
means accepting this license. gaitos itself does not need them: the NAO simulation models
(`urdf/nao_gazebo.urdf`, `mjcf/nao.xml`) use primitive collision shapes; the meshes only give
the looks in RViz and Gazebo.
