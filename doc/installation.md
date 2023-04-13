# 1 ROS2 Installation

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

```bash
mkdir tirrex_ws
cd tirrex_ws
mkdir src
cd src
vcs import < ../romea_tirrex.repos
cd ..
```

### 4.2 Source ROS distro

```bash
source /opt/ros/galactic/setup.bash
```

or

```bash
`source /opt/ros/humble/setup.bash`
```

### 4.3 Workspace compilation

```
rosdep install --from-paths src --ignore-src -r -y --ros-distro=galatic
```

```bash
colcond build 
```

or

```shell
colcon build -parallel-workers NUMBER
```

or

```bash
 colcon build --executor sequential
```



# 5 Usage

Go to workspace 

```bash
source /opt/ros/galactic/setup.bash
source  /usr/share/gazebo/setup.bash
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