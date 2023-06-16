#  ROS2 Installation

Galactic (ubuntu 20.04):

https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html

Humble (ubuntu 22.04)

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html



# 2 Dev tools installation

```bash
sudo apt install git
sudo apt install openssh-server
sudo apt install python3-vcstool
sudo apt install python3-colcon-common-extensions

sudo apt install rosdep
rosdep init
rosdep update --include-eol-distros
```



# 3 GitLab & SSH configuration

Git installation

**Configurer son compte gitlab**

Aller sur la page du gitlab de l'irstea

https://gitlab.irstea.fr/

Se connecter en et aller dans settings.

**Configuration de la connexion ssh**

**Configuration Linux**

Ouvrir un terminal et lancer la commande ci-dessous pour créer une clé ssh en changeant prenom.nom par les votres

```bash
 ssh-keygen -t rsa -C "prenom.nom@inrae.fr" -b 4096
```



`Enter file in which to save the key :` press enter

Un répertoire .ssh est apparu à la racine du répertoire /home/user et contient :

- la clé privée : ssh id_rsa
- la clé publique : id_rsa.pub

Aller dans la page settings de votre compte gitlab. Puis cliquer sur l'onglet *SSH Keys*. Copier le contenu de la clé publique (id_rsa.pub) dans l'encadrement intitulé *Key* et lui donner un nom dans l'encadré *Title*. Enfin cliquer sur le bouton *Add Key* pour charger la clé ssh dans gitlab.



# 4 Tirrex installation

### 4.1 Workspace creation

https://gitlab.irstea.fr/romea_projects/tirrex/tirrex_demo/-/blob/main/tirrex_demo.repos

```bash
mkdir tirrex_ws
cd tirrex_ws
mkdir src
cd src
vcs import < ../romea_tirrex.repos
cd ..
rosdep install --from-paths src --ignore-src -r -y --ros-distro=galatic
```

### 4.3 Workspace compilation

add /usr/share/gazebo/setup.bash in ~/.bashrc

```bash
cd tirrex_ws
source /opt/ros/ros2_distro/setup.bash
export BUILD_WITH_LDMRS_SUPPORT="True"
colcon build 
```

Where ros2_distro can be galactic or humble. It is possible to change the change number of thread used during compilation in using:

```shell
colcon build -parallel-workers NUMBER
```

or

```bash
 colcon build --executor sequential
```



### 4.4 Bridge compilation (Only under ubuntu 20.04)

In order to use alpo and/or campero robot in live, their ros1 bridge must be compiled:

```bash
cd tirrex_ws
source /opt/ros/ros2_distro/setup.bash
source /opt/ros/noetic/setup.bash
catkin build --packages-select alpo_bridge
catkin build --packages-select campero_bridge
```



# 5 Usage

Go to workspace 

```bash
cd tirrex_ws
source /opt/ros/galactic/setup.bash
source /usr/share/gazebo/setup.bash
source install/setup.bash
```

Launch a robot demo

```bash
ros2 launch adap2e_bringup adap2e.launch.py mode:=simulation record:=false
```

or

```bash
ros2 launch alpo_bringup alpo_launch.py mode:=simulation record:=false
```

or

```bash
ros2 launch campero_bringup campero.launch.py mode:=simulation record:=false
```

or

```bash
ros2 launch robufast_bringup robufast.launch.py mode:=simulation record:=false
```

where:

- mode parameter can be set to simulation or live
- record parameter can be set to true or false 

missing dependence plugins gazebo

# 6 Visual Studio Code

### 6.1 installation