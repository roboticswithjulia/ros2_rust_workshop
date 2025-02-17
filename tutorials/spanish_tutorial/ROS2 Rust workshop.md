# Workshop ROS2 Rust: Desarrollo de Aplicaciones Reales en Robótica con ROS2 y Rust

<p style="text-align: center">
  <img src="../images/ROS2_Rust.png" width="700" title="ROS2 RUST logo">
</p>

Rust es un lenguaje que permite crear un software de calidad y eficiente.

ROS2, el marco de trabajo para sistemas operativos robóticos, ahora ofrece la posibilidad de ser utilizado con Rust. Con este proyecto, aprenderás a desarrollar nodos en Rust para tus aplicaciones robóticas reales.

En este workshop, aprenderás lo siguiente: 

* [1. ¿Qué es / Por qué Rust?](#whatisrust)
* [2. Cómo instalar Rust con ROS2](#howtoinstallrust)
* [3. Ejecutar la simulación](#setupworkspace)
* [4. Cómo mover un robot con ROS2 y Rust](#movearobotwithROS2)
* [4.1 Cómo crear un paquete de ROS en Rust](#createarustrospackage)
* [4.2 Consejos básicos de programación en Rust](#basicrustprogrammingtips)
* [4.3 Cómo crear un *subscriber* en Rust](#howtocreateasubscribertoscantopicinrust)
* [4.4 Cómo crear un *publisher* en Rust](#howtocreateapublishertocmdvelinrust)
* [4.5 Cómo crear un *subscriber* y un *publisher* en el mismo nodo](#howtocreatasubandpub)
* [4.6 How to create a Service message](#howtocreateasrvmessage)
* [4.7 How to create a Service](#howtocreateaservice)
* [5. Trabajo futuro](#futurework)

## <a name="whatisrust"></a> 1. ¿Qué es / Por qué Rust?

Rust es un lenguaje de programación de sistemas diseñado para optimizar el rendimiento, la seguridad y la concurrencia. Fue desarrollado originalmente por Mozilla y ahora es mantenido por la Fundación Rust, que incluye partes de código abierto.

<div align="center">
    <img src="../images/Rust_Foundation_logo.png" width="300" alt="The Rust foundation">
</div>

En la siguiente imagen se encuentran algunas características y aspectos clave de Rust:

<div align="center">
    <img src="../images/Rust_diagram.png" width="800" alt="Rust diagram">
</div>

## <a name="howtoinstallrust"></a> 2. Cómo instalar Rust con ROS2:

Los paquetes de ROS2 no se pueden localizar por defecto dentro de los paquetes de Rust. La única forma de trabajar con ello actualmente es instalarlo desde el código fuente o utilizando Docker. 
Dichos recursos se pueden encontrar aquí:
https://github.com/ros2-rust/ros2_rust.


## <a name="setupworkspace"></a> 3. Cómo ejecutar la simulación del robot

1. Ejecutar la simulación en _Gazebo_ en la `Terminal #1`.
> Puede que tarde un poco la primera vez que se ejhecute, esperar a que se abra inicie simulación.


```bash
cd ~/ros2_rust_workshop/ros_ws
source /opt/ros/humble/setup.bash 
colcon build
. install/setup.sh
ros2 launch go2_config gazebo_velodyne.launch.py world:=$(ros2 pkg prefix go2_config)/share/go2_config/worlds/outdoor.world
```

> [!WARNING]  
> **_NOTE:_**  Si se esta usando **Ubuntu 24.04** y devcontainers, puede que Gazebo no se inicie. Para resolver esto ejecuta el siguiente commando en la terminal: `export DISPLAY=:1` antes de volver a iniciar la simulación y ejecute los comandos anteriores.


<div align="center">
    <img src="../images/outdoor_simulation.png" width="900" alt="Rust diagram">
</div>


1. Ver que *tópicos* hay disponibles. Abre la terminal, `terminal #2` y ejecute:

```bash
cd ~/ros2_rust_workshop/ros_ws
source /opt/ros/humble/setup.sh 
. install/setup.sh 
ros2 topic list
```
Salida `terminal #2`:

```bash
/base_to_footprint_pose
/body_pose
/clicked_point
/clock
/cmd_vel
/diagnostics
/dynamic_joint_states
/foot
/foot_contacts
/goal_pose
/hokuyo_frame/scan
/imu/data
/initialpose
/joint_group_effort_controller/controller_state
/joint_group_effort_controller/joint_trajectory
/joint_group_effort_controller/state
/joint_group_effort_controller/transition_event
/joint_states
/joint_states_controller/transition_event
/odom
/odom/ground_truth
/odom/local
/odom/raw
/parameter_events
/performance_metrics
/robot_description
/rosout
/set_pose
/tf
/tf_static
/velodyne_points
```

## <a name="movearobotwithROS2"></a> 4. Cómo mover un robot con ROS2 y Rust

### <a name="createarustrospackage"></a> 4.1 Cómo crear un paquete de ROS en Rust

**Cargo** es el gestor de paquetes y herramienta de construcción para Rust. Facilita la gestión de dependencias, la construcción de proyectos y la ejecución de pruebas. A través de Cargo, puedes compilar tu código, descargar y actualizar paquetes de terceros (llamados _"crates"_), y administrar configuraciones de tu proyecto.

**Crates** es el nombre que se utiliza para referirse a los paquetes de Rust. Un _crate_ es una unidad de código distribuible que puede ser una biblioteca o un ejecutable. Los crates se gestionan a través de Cargo, el gestor de paquetes de Rust, que facilita su publicación, instalación y actualización. En la siguiente página podéis encontrar los _crates_ actualmente disponibles [Crates.io website](https://crates.io/).

Existen dos tipos principales de crates:

- **Crates de biblioteca**: Contienen código que puede ser utilizado por otros crates. No tienen un punto de entrada principal.
- **Crates binarios**: Contienen un punto de entrada principal (main), que puede ser ejecutado como un programa independiente.

Los crates se definen en el archivo *Cargo.toml*, donde se especifican sus dependencias y configuraciones.

<div align="center">
    <img src="../images/rclrs_crates.png" width="700" alt="crate example">
</div>


#### Para crear un nuevo paquete:

```bash
$ cargo new <pkg_name>
```

Ejecuta en la `terminal #2`:
```bash
cd ~/ros2_rust_workshop/ros_ws/src
cargo new rust_apps
```


Cada paquete de Cargo en ROS2 tendrá la siguiente estructura de archivos y carpetas:

- Carpeta `src`: Contiene los archivos fuente (Rust, CPP, Python).
- `Cargo.toml`: Archivo dónde puedes definir las dependencias (_crates_), metadatos y algunas configuraciones del compilador.
- `Cargo.lock`: Contiene información exacta sobre tus dependencias. Es mantenido por Cargo y no debe ser editado manualmente.

Son esenciales, así que recuerda lo siguiente:

- Cada programa ROS2 que quieras ejecutar está organizado en un paquete.
- Cada programa ROS2 que crees debe estar organizado en un paquete.
- Los paquetes son el sistema principal de organización para los programas de ROS2.

Ejecuta en la `Terminal #2`:
```bash
cd ~/ros2_rust_workshop/ros_ws/src/rust_apps
tree -c
```

Salida de la `Terminal #2`:
```bash
.
|-- Cargo.toml
`-- src
    `-- main.rs

1 directory, 2 files
```

Para que ROS reconozca que se trata de un paquete ROS, es necesario agregar un archivo **package.xml**. Este archivo contiene la información de metadatos sobre el paquete, como su nombre, versión, autor, y las dependencias necesarias.

1. Crea un nuevo fichero **package.xml** dentro del directorio _rust_apps_ `~/ros2_rust_workshop/ros_ws/src/rust_apps/package.xml`.

2. Copia el siguiente código en el siguiente fichero `rust_apps/package.xml`.
```xml
<package format="3">
  <name>rust_apps</name>
  <version>0.0.0</version>
  <description>ROS2 Rust main package</description>
  <maintainer email="user@gmail.com">user</maintainer>
  <license>MIT</license>

  <depend>rclrs</depend>

  <export>
    <build_type>ament_cargo</build_type>
  </export>
</package>
```

Los principales atajos de Rust:

- `cargo build`
- `cargo run`
- `cargo install <name of the package>`

> No vamos a usar estos comandos en absoluto ya que estamos usando ROS2, así que utilizaremos los comandos de ROS2.

Para proyectos grandes, es posible que no desees compilar todos los paquetes de inmediato. En su lugar, puedes seguir los siguientes enfoques:

Ejecute en la `Terminal #2`:

```bash
cd ~/ros2_rust_workshop/ros_ws
colcon build --packages-select rust_apps
source install/setup.sh
```

Ahora tenemos el *crate* (paquete de ROS2) preparado para implementar un ROS2 node usando Rust.

### <a name="basicrustprogrammingtips"></a> 4.2 Consejos Básicos de Programación en Rust

#### 4.2.1 Funciones

```rust
fn <function_name>(variable:type){} -> <return_type>
```

```rust
fn main() -> Result<(), Error> {}
```

#### 4.2.2 Mutabilidad

Para crear una nueva variable y asignarle un valor, se utiliza **let**. Si la variable es mutable (es decir, su valor puede cambiar), se debe utilizar **mut**. 

```rust
let mut message = std_msgs::msg::String::default();
```

#### 4.2.3 Pasos para Crear un Nodo de ROS2 en Rust

##### 4.2.3.1 Crear el contexto, el estado compartido entre nodos y entidades similares.

```rust
let context = rclrs::Context::new(env::args())?;
```

##### 4.2.3.2 Crear el nodo.

```rust
pub fn create_node(
    context: &Context,
    node_name: &str
) -> Result<Arc<Node>, RclrsError>
```

```rust
let node = rclrs::create_node(&context, "<node_name>")?;
```

##### 4.2.3.3 Crear un subscriber.

```rust
pub fn create_subscription<T, Args>(
    &self,
    topic: &str,
    qos: QoSProfile,
    callback: impl SubscriptionCallback<T, Args>
) -> Result<Arc<Subscription<T>>, RclrsError>
where
    T: Message,
```

```rust
let _subscription = node.create_subscription::<PointCloud2, _>(
    "velodyne_points",
    rclrs::QOS_PROFILE_DEFAULT,
    move |msg: sensor_msgs::msg::LaserScan| {
        let point_step = msg.point_step as usize; 
        println!("Bytes per point: '{}'", point_step);
    },
)?;
```

##### 4.2.3.4 Crear un publisher.

```rust
pub fn create_publisher<T>(
    &self,
    topic: &str,
    qos: QoSProfile
) -> Result<Arc<Publisher<T>>, RclrsError>
where
    T: Message,
```

```rust
let publisher = node.create_publisher::<Twist>("cmd_vel", rclrs::QOS_PROFILE_DEFAULT)?;
publisher.publish(&cmd_vel_message)?;
```

##### 4.2.3.5 Qué QoSProfile (Perfil de Calidad de Servicio) tenemos implementado?.

- QOS_PROFILE_CLOCK
- QOS_PROFILE_DEFAULT
- QOS_PROFILE_PARAMETERS
- QOS_PROFILE_PARAMETER_EVENTS
- QOS_PROFILE_SENSOR_DATA
- QOS_PROFILE_SERVICES_DEFAULT
- QOS_PROFILE_SYSTEM_DEFAULT

Dado que el tema del QoS (Quality of Service) es complejo y está fuera del alcance de la sesión de aprendizaje actual, utilizaremos el perfil de QoS predeterminado, QOS_PROFILE_DEFAULT. Si deseas aprender más sobre QoS y cómo personalizarlo para tus necesidades específicas, te recomiendo que consultes la página oficial de ROS2 [QoS](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html).

##### 4.2.3.6 ROS spin - usar fuera de un bucle.

```rust
pub fn spin(node: Arc<Node>) -> Result<(), RclrsError>
```

```rust
rclrs::spin(node).map_err(|err| err.into())
```

##### 4.2.3.7 ROS spin_once - usar dentro de un bucle.

```rust
pub fn spin_once(
    node: Arc<Node>,
    timeout: Option<Duration>
) -> Result<(), RclrsError>
```

```rust
rclrs::spin_once(node.clone(), Some(std::time::Duration::from_millis(500)));
```

#### 4.2.4 Creación y uso de estructuras y métodos

```rust
struct Rectangle {
    width: u32,
    height: u32,
}

impl Rectangle {
    fn area(&self) -> u32 {
        self.width * self.height
    }
}

```

### <a name="howtocreateasubscribertoscantopicinrust"></a> 4.3 Cómo crear un suscriptor

#### 4.3.1 Estudio del mensaje que contiene el tópico

Vamos a ver cómo realizar un subscriptor al _tópico_ `/velodyne_points` utilizando Rust. 

1. En primer vamos a subscribirnos al tópico para ver lo que contiene. 

Ejecuta en la `Terminal #2`:

```bash
source /opt/ros/humble/setup.sh
ros2 topic echo /velodyne_points
```
Salida `Terminal #2`:
```bash
header:
  stamp:
    sec: 287
    nanosec: 12000000
  frame_id: velodyne
height: 1
width: 5546
fields:
- name: x
  offset: 0
  datatype: 7
  count: 1
- name: y
  offset: 4
  datatype: 7
  count: 1
- name: z
  offset: 8
  datatype: 7
  count: 1
- name: intensity
  offset: 12
  datatype: 7
  count: 1
- name: ring
  offset: 16
  datatype: 4
  count: 1
- name: time
  offset: 18
  datatype: 7
  count: 1
is_bigendian: false
point_step: 22
row_step: 122012
data:
- 100
- 123
- 51
- 192
- 23
...
```

2. Para poder crear un subscriptor utilizando Rust, tenemos que saber qué tipo de mensaje y atributos contiene el tópico `/velodyne_points` .

2.1 Ejecuta en la `Terminal #2`:
```bash
ros2 topic info /velodyne_points
```
Salida `Terminal #2`:
```bash
Type: sensor_msgs/msg/PointCloud2
Publisher count: 1
Subscription count: 0
```

2.2 Ejecuta en la `Terminal #2`:
```bash
ros2 interface show sensor_msgs/msg/PointCloud2
```
Salida `Terminal #2`:
```bash
# Time of sensor data acquisition, and the coordinate frame ID (for 3d points).
std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields
        uint8 INT8    = 1
        uint8 UINT8   = 2
        uint8 INT16   = 3
        uint8 UINT16  = 4
        uint8 INT32   = 5
        uint8 UINT32  = 6
        uint8 FLOAT32 = 7
        uint8 FLOAT64 = 8
        string name      #
        uint32 offset    #
        uint8  datatype  #
        uint32 count     #

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

```

#### 4.3.2 Implementación del código

1. Cambia el nombre del fichero `main.rs -> scan_subscriber.rs`.

Ejecuta en la `Terminal #2`:

```bash
cd ~/ros2_rust_workshop/ros_ws/src/rust_apps/src
mv main.rs scan_subscriber.rs
```


2. Abrir en el editor de código: `rust_apps/src/scan_subscriber.rs`

```Rust
use std::{
   env,
   f32::consts::PI,
};
use anyhow::{Error, Result};
use sensor_msgs::msg::PointCloud2;
use std::convert::TryInto;

// Constants for obstacle detection
const MAX_DISTANCE_THRESHOLD: f32 = 1.0;  // Maximum distance to detect obstacles
const ANGLE_TOLERANCE: f32 = 0.1;         // Tolerance for angle detection

fn main() -> Result<(), Error> {
   // Initialize ROS2 context and create node
   let context = rclrs::Context::new(env::args())?;
   let node = rclrs::create_node(&context, "scan_subscriber_node")?;

   // Create subscription to velodyne pointcloud topic
   let _subscription = node.create_subscription::<PointCloud2, _>(
       "velodyne_points",
       rclrs::QOS_PROFILE_DEFAULT,
       move |msg: PointCloud2| {
           let point_step = msg.point_step as usize; // Bytes per point
           
           // Process each point in the cloud
           for point in msg.data.chunks(point_step) {
               // Extract x, y coordinates from point data
               let x = f32::from_le_bytes(point[0..4].try_into().unwrap());
               let y = f32::from_le_bytes(point[4..8].try_into().unwrap());
           
               // Calculate angle from x,y coordinates
               let azimuth = y.atan2(x); // Angle in radians

               // Check LEFT side obstacles
               if y.abs() < MAX_DISTANCE_THRESHOLD && (azimuth < PI/2.0 + ANGLE_TOLERANCE) && (azimuth > PI/2.0 - ANGLE_TOLERANCE) {
                   println!("Obstacle detected at LEFT: Orientation {:.2} [rad] and distance {:.2} [m] ", azimuth, y);
                   break;
               }
               // Check RIGHT side obstacles
               if y.abs() < MAX_DISTANCE_THRESHOLD && (azimuth < -PI/2.0 + ANGLE_TOLERANCE) && (azimuth > -PI/2.0 - ANGLE_TOLERANCE) { 
                   println!("Obstacle detected at RIGHT: Orientation {:.2} [rad] and distance {:.2} [m] ", azimuth, y);
                   break;
               }
               // Check FRONT obstacles
               if x.abs() < MAX_DISTANCE_THRESHOLD && azimuth.abs() < ANGLE_TOLERANCE {
                   println!("Obstacle detected at x FRONT: orientation {:.2} [rad] and distance {:.2} [m]", azimuth, x);
                   break;
               }
               // Check BACK obstacles
               if x.abs() < MAX_DISTANCE_THRESHOLD && azimuth.abs() < (PI + ANGLE_TOLERANCE) && (azimuth.abs() > PI - ANGLE_TOLERANCE) {
                   println!("Obstacle detected at x BACK: orientation {:.2} [rad] and distance {:.2} [m]", azimuth, x);
                   break;
               }  
           }
       },
   )?;

   // Start ROS2 spin loop
   rclrs::spin(node).map_err(|err| err.into())
}

```
3. Añadir las dependencias de ROS2 manualmente: Añadir **sensor_msgs** en el archivo **package.xml**.

```xml
  <package format="3">
  <name>rust_apps</name>
  <version>0.0.0</version>
  <description>ROS2 Rust main package</description>
  <maintainer email="user@gmail.com">user</maintainer>
  <license>MIT</license>

  <depend>rclrs</depend>
  <depend>sensor_msgs</depend>  
  <export>
    <build_type>ament_cargo</build_type>
  </export>
</package>
```

4. Añadir dependencias con otros _crates_ y enlazar el archivo con el nodo de ROS2 en el fichero `rust_apps/cargo.toml`.

```toml
[package]
name = "rust_apps"
version = "0.1.0"
edition = "2021"

# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

[dependencies.sensor_msgs]
sensor_msgs = "*"

[[bin]]
name = "scan_subscriber_node"
path = "src/scan_subscriber.rs"
```

6. Añadir las dependencias de forma automática:

Ejecuta en la `Terminal #2`:
```bash
cd ~/ros2_rust_workshop/ros_ws/src/rust_apps
cargo add anyhow
cargo add rclrs
```

7. Construir el nodo:

Ejecuta en la `Terminal #2`:
```bash
cd ~/ros2_rust_workshop/ros_ws
colcon build --packages-select rust_apps
```

#### 4.3.3 Ejecución del código

Ejecuta en la `Terminal #2`:

```bash
cd ~/ros2_rust_workshop/ros_ws
source install/setup.sh
ros2 run rust_apps scan_subscriber_node
```

Output:
```
Obstacle detected at x BACK: orientation -3.07 [rad] and distance -0.91 [m]
Obstacle detected at x FRONT: orientation 0.09 [rad] and distance 0.88 [m]
Obstacle detected at LEFT: Orientation 1.48 [rad] and distance 0.96 [m] 
Obstacle detected at LEFT: Orientation 1.48 [rad] and distance 0.90 [m] 
Obstacle detected at x FRONT: orientation -0.09 [rad] and distance 0.94 [m]
Obstacle detected at x BACK: orientation -3.10 [rad] and distance -0.88 [m]
Obstacle detected at x BACK: orientation 3.10 [rad] and distance -0.92 [m]
Obstacle detected at x BACK: orientation -3.14 [rad] and distance -0.90 [m]
Obstacle detected at x FRONT: orientation 0.08 [rad] and distance 0.89 [m]
Obstacle detected at RIGHT: Orientation -1.51 [rad] and distance -0.87 [m] 
...
```


<div align="center">
    <img src="../videos/velodyne_subscriber.gif" width="800" alt="velodyne subscriber">
</div>



### <a name="howtocreateapublishertocmdvelinrust"></a> 4.4 Cómo crear un cmd_vel Publisher en Rust

#### 4.4.1 Implementación del código

1. Crea un nuevo fichero dentro del paquete de Rust *rust_apps* con el nombre *cmd_vel_publisher.rs* y pega el siguiente código:

IDE(Editor de código): `rust_apps/src/cmd_vel_publisher.rs`
```rust
use std::env;
use anyhow::{Error, Result};
use geometry_msgs::msg::Twist as Twist;

// Constants for velocity control
const INITIAL_VELOCITY: f64 = 1.0;
const VELOCITY_DECREASE: f64 = 0.05;
const VELOCITY_THRESHOLD: f64 = 1.0;

fn main() -> Result<(), Error> {
    // Initialize ROS2 context
    let context = rclrs::Context::new(env::args())?;
    // Create publisher node
    let node = rclrs::create_node(&context, "cmd_vel_publisher_node")?; 
    // Create Twist message publisher
    let publisher = node.create_publisher::<Twist>("cmd_vel", rclrs::QOS_PROFILE_DEFAULT)?;
    // Initialize Twist message and velocity
    let mut cmd_vel_message = Twist::default();
    let mut velocity = INITIAL_VELOCITY;
    // Main loop
    while context.ok() {
        // Set linear and angular velocities
        cmd_vel_message.linear.x = velocity;
        cmd_vel_message.linear.y = velocity;
        cmd_vel_message.angular.z = 0.0;
        // Reset velocity if below threshold, otherwise decrease
        if velocity < VELOCITY_THRESHOLD*(-1.0) {
            velocity = VELOCITY_THRESHOLD
        } else {
            velocity -= VELOCITY_DECREASE
        };
        // Log and publish message
        println!("Moving velocity lineal x: {:.2} and angular z: {:.2} m/s.",
                cmd_vel_message.linear.x, cmd_vel_message.angular.z);
        publisher.publish(&cmd_vel_message)?;
        // Sleep for 500ms
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    Ok(())
}
```

2. Añade la dependencia *geometry_msgs* y añade el nuevo nodo llamado *cmd_vel_publisher* dentro de *Cargo.toml*:

Editor: `rust_apps/Cargo.toml`
```toml

[package]
name = "rust_apps"
version = "0.1.0"
edition = "2021"

[dependencies]
anyhow = "1.0.95"
rclrs = "0.4.1"

# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

[dependencies.sensor_msgs]
sensor_msgs = "*"

[dependencies.geometry_msgs]
geometry_msgs = "*"

[[bin]]
name = "scan_subscriber_node"
path = "src/scan_subscriber.rs"

[[bin]]
name = "cmd_vel_publisher_node"
path = "src/cmd_vel_publisher.rs"


```

Editor: `rust_apps/package.xml`
```xml
<package format="3">
  <name>rust_apps</name>
  <version>0.0.0</version>
  <description>ROS2 Rust main package</description>
  <maintainer email="user@gmail.com">user</maintainer>
  <license>TBD</license>

  <depend>rclrs</depend>
  <depend>sensor_msgs</depend>  
  <depend>geometry_msgs</depend>  
  <export>
    <build_type>ament_cargo</build_type>
  </export>
</package>
```

3. Construya el paquete de ROS2:

Ejecuta en la `Terminal #2`:
```bash
cd ~/ros2_rust_workshop/ros_ws
colcon build --packages-select rust_apps
```

#### 4.4.2 Ejecución del código

Ejecuta en la `Terminal #2`:
```bash
cd ~/ros2_rust_workshop/ros_ws
source install/setup.sh
```
4. Abre la ventana de *gazebo* y busca al robot go2 y después executa el nodo `cmd_vel_publisher`.

Ejecuta en la `Terminal #2`:
```
source /opt/ros/humble/setup.sh 
ros2 run rust_apps cmd_vel_publisher_node
```


<div align="center">
    <img src="../videos/cmd_vel_publisher.gif" width="800" alt="cmd vel publisher">
</div>


Si se quiere parar el robot, puedes terminal el programa con `Ctrl+C` y ejecutar después la siguiente línea de código:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

### <a name="howtocreatasubandpub"></a> 4.5 Cómo crear un Subscriber y un Publisher en el mismo nodo

#### 4.5.1 Implementación del código

1. Los proyectos reales van más allá de los modelos de publicación y suscripción simples. Colaboremos para crear una estructura de Rust que incluya tanto un publicador como un suscriptor. Esta estructura permitirá que el robot navegue de forma autónoma por el entorno de simulación, evitando colisiones de forma independiente.

Editor de código: `rust_apps/src/obstacle_avoidance.rs`

``` rust
// Dependencies for ROS2, synchronization and geometry types
use std::{env, sync::{Arc, Mutex}};
use sensor_msgs::msg::PointCloud2 as PointCloud2;
use geometry_msgs::msg::Twist as Twist;
use anyhow::{Error, Result};
use std::f32::consts::PI;

// Configuration constants for robot movement and obstacle detection
const ANGULAR_SPEED: f64 = 0.5;      // Turning speed in rad/s
const LINEAR_SPEED: f64= 2.0;        // Forward/backward speed in m/s
const MAX_DISTANCE_THRESHOLD: f32 = 1.0;  // Minimum distance to obstacle in meters
const ANGLE_TOLERANCE: f32 = 0.1;    // Angle tolerance for direction detection

// Main struct managing obstacle avoidance behavior
struct ObstacleAvoidance {
    _subscription: Arc<rclrs::Subscription<PointCloud2>>,  // LiDAR data subscriber
    publication: Arc<rclrs::Publisher<Twist>>,            // Velocity command publisher
    twist_msg: Arc<Mutex<Twist>>                         // Thread-safe velocity message
}

impl ObstacleAvoidance {
    // Constructor for ObstacleAvoidance system
    pub fn new(node: &rclrs::Node) -> Result<Self, rclrs::RclrsError> {
        let twist_msg = Arc::new(Mutex::new(Twist::default()));
        let publication = node.create_publisher::<Twist>("cmd_vel", rclrs::QOS_PROFILE_DEFAULT)?;
        let twist_msg_clone = Arc::clone(&twist_msg);

        // Subscribe to LiDAR point cloud data
        let _subscription = node.create_subscription::<sensor_msgs::msg::PointCloud2, _>(
            "velodyne_points",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: sensor_msgs::msg::PointCloud2| {
                let mut twist_msg = twist_msg_clone.lock().unwrap();
                let point_step = msg.point_step as usize;

                // Process each point in the cloud
                for point in msg.data.chunks(point_step) {
                    // Extract point coordinates
                    let x = f32::from_le_bytes(point[0..4].try_into().unwrap());
                    let y = f32::from_le_bytes(point[4..8].try_into().unwrap());
                    
                    // Calculate point angle relative to robot
                    let azimuth = y.atan2(x);

                    // Default movement: forward
                    twist_msg.linear.x = LINEAR_SPEED; 
                    twist_msg.linear.y = 0.0;
                    twist_msg.angular.z = 0.0; 
    
                    // Check for obstacles in different directions and adjust movement

                    // Left obstacle detection
                    if y.abs() < MAX_DISTANCE_THRESHOLD && (azimuth < PI/2.0 + ANGLE_TOLERANCE) && (azimuth > PI/2.0 - ANGLE_TOLERANCE) {
                        println!("Obstacle detected at LEFT: orientation {:.2} [rad], distance {:.2} [m], turning RIGHT", azimuth, y);
                        twist_msg.linear.x = 0.0; 
                        twist_msg.linear.y = 0.0; 
                        twist_msg.angular.z = -ANGULAR_SPEED;
                        break;
                    }

                    // Right obstacle detection
                    if y.abs() < MAX_DISTANCE_THRESHOLD && (azimuth < -PI/2.0 + ANGLE_TOLERANCE) && (azimuth > -PI/2.0 - ANGLE_TOLERANCE) { 
                        println!("Obstacle detected at RIGHT: orientation {:.2} [rad], distance {:.2} [m], turning LEFT", azimuth, y);
                        twist_msg.linear.x = 0.0; 
                        twist_msg.linear.y = 0.0; 
                        twist_msg.angular.z = ANGULAR_SPEED;
                        break;
                    }

                    // Front obstacle detection
                    if x.abs() < MAX_DISTANCE_THRESHOLD && azimuth.abs() < ANGLE_TOLERANCE {
                        println!("Obstacle detected at FRONT: orientation {:.2} [rad], distance {:.2} [m], going BACKWARD", azimuth, x);
                        twist_msg.linear.x = -LINEAR_SPEED;
                        twist_msg.linear.y = 0.0;
                        twist_msg.angular.z = ANGULAR_SPEED;
                        break;
                    }

                    // Back obstacle detection
                    if x.abs() < MAX_DISTANCE_THRESHOLD && azimuth.abs() < (PI + ANGLE_TOLERANCE) && (azimuth.abs() > PI - ANGLE_TOLERANCE) {
                        println!("Obstacle detected at BACK: orientation {:.2} [rad], distance {:.2} [m], going FORWARD", azimuth, x);
                        twist_msg.linear.x = LINEAR_SPEED;
                        twist_msg.linear.y = 0.0;
                        twist_msg.angular.z = ANGULAR_SPEED;
                        break;
                    }       
                } 
            },
        )?;

        Ok(Self{_subscription, publication, twist_msg})
    }

    // Publish current velocity commands to the robot
    pub fn publish(&self) {
        let twist_msg = self.twist_msg.lock().unwrap();
        let _ = self.publication.publish(&*twist_msg);
    }
}

// Initialize ROS2 node and run obstacle avoidance loop
fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;
    let node = rclrs::create_node(&context, "obstacle_avoidance_node")?;
    let subscriber_node_one = ObstacleAvoidance::new(&node)?;

    while context.ok() {
        subscriber_node_one.publish();
        let _ = rclrs::spin_once(node.clone(), Some(std::time::Duration::from_millis(10)));
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    Ok(())
}
```
2. Añadir el nuevo nodo ejecutable en `Cargo.toml`

```toml
[package]
name = "rust_apps"
version = "0.1.0"
edition = "2021"

[dependencies]
anyhow = "1.0.95"
rclrs = "0.4.1"

# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

[dependencies.sensor_msgs]
sensor_msgs = "*"

[dependencies.geometry_msgs]
geometry_msgs = "*"

[[bin]]
name = "scan_subscriber_node"
path = "src/scan_subscriber.rs"

[[bin]]
name = "cmd_vel_publisher_node"
path = "src/cmd_vel_publisher.rs"

[[bin]]
name = "obstacle_avoidance_node"
path = "src/obstacle_avoidance.rs"
```
Ejecuta en la `Terminal #2`:

```bash
cd ~/ros2_rust_workshop/ros_ws
colcon build --packages-select rust_apps
```


#### 4.5.2 Ejecución del código

Ejecuta en la `Terminal #2`:

```bash
source install/setup.sh
ros2 run rust_apps obstacle_avoidance_node
```
<div align="center">
    <img src="../videos/obstacle_avoidance.gif" width="800" alt="obstacle avoidance">
</div>


Si se quiere parar el robot, puedes terminal el programa con `Ctrl+C` y ejecutar después la siguiente línea de código:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```


### <a name="howtocreateasrvmessage"></a> 4.6 Cómo crear un mensaje Command.srv

1. Crea un nuevo crate llamado `rust_msgs`
```bash
cd ~/ros2_rust_workshop/ros_ws/src
cargo new rust_msgs
```

2. Elimina el fichero `Cargo.toml`
```bash
cd ~/ros2_rust_workshop/ros_ws/src/rust_msgs
rm -r Cargo.toml
```

3. Crea una carpeta `srv` y un fichero `Command.srv`

```bash
cd ~/ros2_rust_workshop/ros_ws/src/rust_msgs
mkdir srv
cd srv
touch Command.srv
```
4. Copia el siguiente mensaje en `~/ros2_rust_workshop/ros_ws/src/rust_msgs/Command.srv`

```txt
# Command

int32 STOP = 0
int32 START = 1

int32 command

---
bool success
string message
```
5. Crea un fichero `CMakeLists.txt` dentro de `~/ros2_rust_workshop/ros_ws/src/rust_msgs` y copia el siguiente código:

```cmake
cmake_minimum_required(VERSION 3.5)

project(rust_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  "srv/Command.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
```
6. Crea un `package.xml` dentro de  `~/ros2_rust_workshop/ros_ws/src/rust_msgs` y añade las respectivas dependencias `ament`, `rosidl_default_generators`, `rosidl_default_runtime` y `rosidl_interface_packages`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rust_msgs</name>
  <version>0.4.1</version>
  <description>A package containing some example message definitions.</description>
  <maintainer email="user@gmail.com">user</maintainer>
  <license>TBD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <test_depend>ament_lint_common</test_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
5. Compila y haz un source:

```bash
cd ~/ros2_rust_workshop/ros_ws
colcon build --packages-select rust_msgs
source install/setup.sh
```
6. Verifica que el mensaje `Command.srv` ha sido creado:
```bash
ros2 interface show rust_msgs/srv/Command
```
Salida: 
```bash
# Command

int32 STOP = 0
int32 START = 1

int32 command

---
bool success
string message
```


### <a name="howtocreateaservice"></a> 4.7 Cómo crear un Servicio:

#### 4.7.1 Cómo crear un servicio servidor:

1. Crear un fichero en  `~/ros2_rust_workshop/ros_ws/src/rust_apps/src` llamado `cmd_service_server.rs`:

```bash
cd ~/ros2_rust_workshop/ros_ws/src/rust_apps/src
touch cmd_service_server.rs
```

2. Pegar el siguiente código en `~/ros2_rust_workshop/ros_ws/src/rust_apps/src/cmd_service_server.rs`

```rust
use std::env;
use anyhow::{Error, Result};
use std::sync::Arc;
use geometry_msgs::msg::{Twist, Vector3}; // Simplify imports for better clarity

// Command constants for better readability and to avoid magic numbers
const STOP: i32 = 0;
const START: i32 = 1;

/// Handles incoming service requests to control the robot
///
/// # Arguments
/// * `_request_header` - Metadata for the request (not used here but required by ROS2 service definition).
/// * `request` - The service request containing the command (STOP or START).
/// * `publisher` - A shared publisher to send velocity commands to the robot.
///
/// # Returns
/// A response indicating success or failure of the command execution.
fn handle_service(
    _request_header: &rclrs::rmw_request_id_t,
    request: rust_msgs::srv::Command_Request,
    publisher: Arc<rclrs::Publisher<Twist>>,
) -> rust_msgs::srv::Command_Response {
    // Initialize a default Twist message with zero velocities
    let mut twist_msg = Twist {
        linear: Vector3 { x: 0.0, y: 0.0, z: 0.0 },
        angular: Vector3 { x: 0.0, y: 0.0, z: 0.0 },
    };

    match request.command {
        STOP => {
            // Publish a stop command (zero velocities)
            if let Err(err) = publisher.publish(&twist_msg) {
                eprintln!("Failed to publish STOP command: {}", err);
                return rust_msgs::srv::Command_Response {
                    message: "Failed to stop robot".to_string(),
                    success: false,
                };
            }
            rust_msgs::srv::Command_Response {
                message: "Stopping Robot".to_string(),
                success: true,
            }
        }
        START => {
            // Update the Twist message to move forward and rotate slightly
            twist_msg.linear.x = 1.0; // Move forward
            twist_msg.angular.z = 0.1; // Slight rotation

            // Publish the start command
            if let Err(err) = publisher.publish(&twist_msg) {
                eprintln!("Failed to publish START command: {}", err);
                return rust_msgs::srv::Command_Response {
                    message: "Failed to start robot".to_string(),
                    success: false,
                };
            }
            rust_msgs::srv::Command_Response {
                message: "Starting Robot".to_string(),
                success: true,
            }
        }
        _ => {
            // Handle invalid commands
            rust_msgs::srv::Command_Response {
                message: "Invalid command".to_string(),
                success: false,
            }
        }
    }
}

fn main() -> Result<(), Error> {
    // Initialize the ROS2 context
    let context = rclrs::Context::new(env::args())?;

    // Create a new ROS2 node
    let node = rclrs::create_node(&context, "cmd_service_server")?;

    // Create a publisher for the `cmd_vel` topic
    let cmd_vel_publisher = node.create_publisher::<Twist>("cmd_vel", rclrs::QOS_PROFILE_DEFAULT)?;

    // Create a service for handling robot commands
    let _server = node.create_service::<rust_msgs::srv::Command, _>("command", move |req_header, request| {
        // Use a closure to pass the publisher to the service handler
        handle_service(req_header, request, cmd_vel_publisher.clone())
    })?;

    println!("Starting server. Waiting for requests...");

    // Spin the node to process incoming service requests
    rclrs::spin(node).map_err(|err| err.into())
}
```

3. Añadir un nuevo ejecutable en `~/ros2_rust_workshop/ros_ws/src/rust_apps/Cargo.toml`

```toml
(...)
[dependencies.rust_msgs]
rust_msgs = "*"

(...)
[[bin]]
name = "cmd_service_server"
path = "src/cmd_service_server.rs"
```
4. En la `terminal #2` compila los últimos cambios y ejecuta el servidor:

```bash
cd ~/ros2_rust_workshop/ros_ws
colcon build --packages-select rust_apps
source install/setup.bash
ros2 run rust_apps cmd_service_server
```

5. Abrir una nueva terminal y realice solicitudes de cliente ejecutando los siguientes comandos: 

5.1. El robot GO2 empiezará a moverse después de ejecutar el siguiente comando:

```bash
cd ~/ros2_rust_workshop/ros_ws
source install/setup.sh
ros2 service call /command rust_msgs/srv/Command "{command: 1}"
```
Salida:
```
requester: making request: rust_msgs.srv.Command_Request(command=1)

response:
rust_msgs.srv.Command_Response(success=True, message='Starting Robot')
```

5.2 El robot GO2 dejará de moverse después de enviar la siguiente solicitud de comando:

```bash
ros2 service call /command rust_msgs/srv/Command "{command: 0}"
```
Salida:
```
requester: making request: rust_msgs.srv.Command_Request(command=0)

response:
rust_msgs.srv.Command_Response(success=True, message='Stopping Robot')
```

5.3 Detener el servidor de servicio:

Escribe `CTRL+C` en la `terminal #2`


#### 4.7.2 Cómo crear un cliente de servicio:

1. Crea dos archivos diferentes en `~/ros2_rust_workshop/ros_ws/src/rust_apps/src`

```bash
cd ~/ros2_rust_workshop/ros_ws/src/rust_apps/src
touch cmd_service_client_start.rs
touch cmd_service_client_stop.rs

```
2. Copia el siguiente código en `~/ros2_rust_workshop/ros_ws/src/rust_apps/src/cmd_service_client_start.rs`
```rust
use std::env;

use anyhow::{Error, Result};
const START: i32 = 1;

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let node = rclrs::create_node(&context, "cmd_service_client_stop")?;

    let client = node.create_client::<rust_msgs::srv::Command>("command")?;

    let request = rust_msgs::srv::Command_Request { command: START };

    println!("Starting client");

    while !client.service_is_ready()? {
        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    client.async_send_request_with_callback(
        &request,
        move |response: rust_msgs::srv::Command_Response| {
            println!(
                "Request command is {} and response is {} with message {}",
                request.command, response.success, response.message
            );
        }
    )?;

    std::thread::sleep(std::time::Duration::from_millis(500));

    println!("Waiting for response");
    rclrs::spin(node).map_err(|err| err.into())
}
```

3. Copia el siguiente código en `~/ros2_rust_workshop/ros_ws/src/rust_apps/src/cmd_service_client_stop.rs`
```rust
use std::env;

use anyhow::{Error, Result};
const STOP: i32 = 0;

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let node = rclrs::create_node(&context, "cmd_service_client_stop")?;

    let client = node.create_client::<rust_msgs::srv::Command>("command")?;

    let request = rust_msgs::srv::Command_Request { command: STOP };

    println!("Starting client");

    while !client.service_is_ready()? {
        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    client.async_send_request_with_callback(
        &request,
        move |response: rust_msgs::srv::Command_Response| {
            println!(
                "Request command is {} and response is {} with message {}",
                request.command, response.success, response.message
            );
        }
    )?;

    std::thread::sleep(std::time::Duration::from_millis(500));

    println!("Waiting for response");
    rclrs::spin(node).map_err(|err| err.into())
}
```
4. Agregue dos nuevos ejecutables en `~/ros2_rust_workshop/ros_ws/src/rust_apps/Cargo.toml`

```toml
(...)

[[bin]]
name = "cmd_service_client_start"
path = "src/cmd_service_client_start.rs"

[[bin]]
name = "cmd_service_client_stop"
path = "src/cmd_service_client_stop.rs"
```
5. Usando la `terminal #2` ejecuta el servidor:
```bash
cd ~/ros2_rust_workshop/ros_ws
colcon build --packages-select rust_apps
source install/setup.bash
ros2 run rust_apps cmd_service_server
```
6. Usando la `terminal #3` compila los últimos cambios y ejecuta el primer cliente:
```bash
cd ~/ros2_rust_workshop/ros_ws
source install/setup.bash
ros2 run rust_apps cmd_service_client_start
```

El robot Go2 debería empezar a caminar.

7. Usando la `terminal #4` compila los úlitmos cambios y ejecuta el segundo:
```bash
cd ~/ros2_rust_workshop/ros_ws
source install/setup.bash
ros2 run rust_apps cmd_service_client_stop
```
El robot Go2 debería dejar de caminar.

>Puedes proceder a terminar con todas las terminales con `Ctrl+C`


## 5. Trabajo futuro

Ahora estás preparado para crear nodos en Rust. Para mejorar aún más tu competencia, te recomiendo que revises los siguientes cursos y documentación:

ROS2 Rust:
- [Repositorio de ROS2 Rust](https://github.com/ros2-rust/ros2_rust)
- [ROS2 Basics in 3 Days (Rust)](https://app.theconstruct.ai/courses/168)
- [ROS2 con Rust | ROS2 Developers Open Class - The Construct](https://youtu.be/ShCnUasOBzU?feature=shared)
- [Construye un nodo de ROS2 con Rust - Mike](https://www.youtube.com/watch?v=U5wHiZpNdvg)
- [Estado actual de las librerias de ROS2 Rust - ROS discourse](https://discourse.ros.org/t/current-state-of-rust-client-libraries-which-one-to-use-ros2-client-rus2-ros2-rust-rclrust-rosrust-or-r2r/39119)

Aprender Rust:
- [Libro de Rust](https://doc.rust-lang.org/book/)
- [Los 9 mejores cursos y libros de programación en Rust para principiantes en 2024](https://medium.com/javarevisited/7-best-rust-programming-courses-and-books-for-beginners-in-2021-2ed2311af46c)

<br>
<br>

<br>







<div align="center">
    <img src="../images/thankyou.png" width="900" alt="thank you">
</div>
