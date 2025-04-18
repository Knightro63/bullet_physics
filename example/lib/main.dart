import 'dart:async';
import 'dart:math' as m;
import 'package:flutter/material.dart';
import 'package:bullet_physics/bullet_physics.dart' as bullet;
import 'package:three_js/three_js.dart' as three;
import 'package:vector_math/vector_math.dart' as math;

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({Key? key}) : super(key: key);

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'Flutter Demo',
      theme: ThemeData(
        primarySwatch: Colors.blue,
      ),
      home: const BasicPhysics(),
    );
  }
}

extension on math.Quaternion{
  three.Quaternion toQuaternion(){
    return three.Quaternion(x,y,z,w);
  }
}
extension on math.Vector3{
  three.Vector3 toVector3(){
    return three.Vector3(x,y,z);
  }
}
class BasicPhysics extends StatefulWidget {
  const BasicPhysics({
    Key? key,
    this.offset = const Offset(0,0)
  }) : super(key: key);

  final Offset offset;

  @override
  _BasicPhysicsPageState createState() => _BasicPhysicsPageState();
}

class _BasicPhysicsPageState extends State<BasicPhysics> {
  late three.ThreeJS threeJs;
  late three.OrbitControls controls;

  List<three.Mesh> meshs = [];
  List<three.Mesh> grounds = [];
  
  Map<String,three.BufferGeometry> geos = {};
  Map<String,three.Material> mats = {};

  //oimo var
  late bullet.DynamicsWorld world;
  List<bullet.RigidBody> bodys = [];

  double toRad = 0.0174532925199432957;
  int type = 1;
  int max = 2;

  @override
  void initState() {
    threeJs = three.ThreeJS(
      onSetupComplete: (){setState(() {});},
      setup: setup,
      settings: three.Settings(
        useOpenGL: true
      )
    );
    super.initState();
  }
  @override
  void dispose() {
    threeJs.dispose();
    controls.dispose();
    super.dispose();
  }
  
  Future<void> setup() async {
    threeJs.scene = three.Scene();

    threeJs.camera = three.PerspectiveCamera(60, threeJs.width / threeJs.height, 1, 10000);
    threeJs.camera.position.setValues(0,160,400);

    controls = three.OrbitControls(threeJs.camera, threeJs.globalKey);
    //controls.target.set(0,20,0);
    //controls.update();
    
    threeJs.scene.add(three.AmbientLight( 0x3D4143, 1.0) );
    three.DirectionalLight light = three.DirectionalLight( 0xffffff , 1.4);
    light.position.setValues( 300, 1000, 500 );
    light.target!.position.setValues( 0, 0, 0 );
    light.castShadow = true;

    const double d = 300;
    light.shadow!.camera = three.OrthographicCamera( -d, d, d, -d,  500, 1600 );
    light.shadow!.bias = 0.0001;
    light.shadow!.mapSize.width = light.shadow!.mapSize.height = 1024;

    threeJs.scene.add( light );

    // background
    three.BufferGeometry buffgeoBack = three.IcosahedronGeometry(3000,2);
    three.Mesh back = three.Mesh( 
      buffgeoBack, 
      three.MeshLambertMaterial()
    );
    threeJs.scene.add( back );

    // geometrys
    geos['sphere'] = three.SphereGeometry(1,16,10);
    geos['box'] =  three.BoxGeometry(1,1,1);
    geos['cylinder'] = three.CylinderGeometry(1,1,1);
    
    // materials
    mats['sph']    = three.MeshPhongMaterial.fromMap({'shininess': 10, 'name':'sph'});
    
    mats['box']    = three.MeshPhongMaterial.fromMap({'shininess': 10, 'name':'box'});
    mats['cyl']    = three.MeshPhongMaterial.fromMap({'shininess': 10, 'name':'cyl'});
    mats['ssph']   = three.MeshPhongMaterial.fromMap({'shininess': 10, 'name':'ssph'});
    mats['sbox']   = three.MeshPhongMaterial.fromMap({'shininess': 10, 'name':'sbox'});
    mats['scyl']   = three.MeshPhongMaterial.fromMap({'shininess': 10, 'name':'scyl'});
    mats['ground'] = three.MeshPhongMaterial.fromMap({'shininess': 10, 'color':0x3D4143, 'transparent':true, 'opacity':0.5});
  
    initBulletPhysics();

    threeJs.addAnimationEvent((dt){
      updateBulletPhysics();
      controls.update();
    });
  }

  void addStaticBox(List<double> size,List<double> position,List<double> rotation) {
    three.Mesh mesh = three.Mesh( geos['box'], mats['ground'] );
    mesh.scale.setValues( size[0], size[1], size[2] );
    mesh.position.setValues( position[0], position[1], position[2] );
    mesh.rotation.set( rotation[0]*toRad, rotation[1]*toRad, rotation[2]*toRad );
    threeJs.scene.add( mesh );
    grounds.add(mesh);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
  }

  void clearMesh(){
    for(int i = 0; i < meshs.length;i++){ 
      threeJs.scene.remove(meshs[i]);
    }

    for(int i = 0; i < grounds.length;i++){ 
      threeJs.scene.remove(grounds[ i ]);
    }
    grounds = [];
    meshs = [];
  }

  //----------------------------------
  //  bullet PHYSICS
  //----------------------------------

  void initBulletPhysics(){
  	// collision configuration contains default setup for memory, collision setup
		bullet.DefaultCollisionConfiguration collisionConfiguration = bullet.DefaultCollisionConfiguration();
		// use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		bullet.CollisionDispatcher dispatcher = bullet.CollisionDispatcher(collisionConfiguration);
		bullet.BroadphaseInterface broadphase = bullet.DbvtBroadphase();
		// math.Vector3 worldAabbMin = math.Vector3(-10000, -10000, -10000);
		// math.Vector3 worldAabbMax = math.Vector3(10000, 10000, 10000);
		// int maxProxies = 1024;
		//bullet.AxisSweep3 broadphase = bullet.AxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

		bullet.SequentialImpulseConstraintSolver solver = bullet.SequentialImpulseConstraintSolver();
    world = bullet.DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    world.setGravity(math.Vector3(0, -200, 0));
    
    populate(type);
  }

  void populate(n) {
    if(n==1){ 
      type = 1;
    }
    else if(n==2){ 
      type = 2;
    }
    else if(n==3){ 
      type = 3;
    }
    else if(n==4) {
      type = 4;
    }

    // reset old
    clearMesh();
    bodys=[];
    world.addRigidBody(bullet.RigidBody.constructor(
      bullet.RigidBodyConstructionInfo(
        0,
        bullet.DefaultMotionState(
          bullet.Transform()
          ..origin.setValues(0,0,0)
        ),
        bullet.BoxShape(math.Vector3(40.0/2, 40.0/2, 390.0/2)),
      )
    ));
    world.addRigidBody(bullet.RigidBody.constructor(
      bullet.RigidBodyConstructionInfo(
        0,
        bullet.DefaultMotionState(
          bullet.Transform()
          ..origin.setValues(180.0,20.0,0.0)
        ),
        bullet.BoxShape(math.Vector3(40.0/2, 40.0/2, 390.0/2)),
      )
    ));
    world.addRigidBody(bullet.RigidBody.constructor(
      bullet.RigidBodyConstructionInfo(
        0,
        bullet.DefaultMotionState(
          bullet.Transform()
          ..origin.setValues(0.0,-40.0,0.0)
        ),
        //bullet.SphereShape(100),
        //bullet.StaticPlaneShape(math.Vector3(1,0,0),1),
        bullet.BoxShape(math.Vector3(400.0/2, 80.0/2, 400.0/2)),
      )
    ));
    addStaticBox([40, 40, 390], [-180,20,0], [0,0,0]);
    addStaticBox([40, 40, 390], [180,20,0], [0,0,0]);
    addStaticBox([400, 80, 400], [0,-40,0], [0,0,0]);

    //add object
    double x, y, z, w, h, d;
    int t;
    for(int i = 0; i < max;i++){
      if(type==4) {
        t = (m.Random().nextDouble() *3).floor()+1;
      }
      else {
        t = type;
      }
      x = -100 + m.Random().nextDouble() *200;
      z = -100 + m.Random().nextDouble() *200;
      y = 100 + m.Random().nextDouble() *1000;
      w = 10 + m.Random().nextDouble() *10;
      h = 10 + m.Random().nextDouble() *10;
      d = 10 + m.Random().nextDouble() *10;
      three.Color randColor = three.Color.fromHex32((m.Random().nextDouble()  * 0xFFFFFF).toInt());

      if(t==1){
        three.Material mat = mats['sph']!;
        mat.color = randColor;
        bullet.RigidBody sbody = bullet.RigidBody.constructor(
          bullet.RigidBodyConstructionInfo(
            1,
            bullet.DefaultMotionState(
              bullet.Transform()
              ..origin.setValues(x,y,z)
            ),
            bullet.SphereShape(w*0.5)..calculateLocalInertia(1, math.Vector3.zero()),
            math.Vector3.zero()
          )
        );
        bodys.add(sbody);
        world.addRigidBody(sbody);
        meshs.add(three.Mesh( geos['sphere'], mat));
        meshs[i].scale.setValues( w*0.5, w*0.5, w*0.5 );
      } 
      else if(t==2){
        three.Material mat = mats['box']!;
        mat.color = randColor;
        bullet.RigidBody sbody = bullet.RigidBody.constructor(
          bullet.RigidBodyConstructionInfo(
            1,
            bullet.DefaultMotionState(
              bullet.Transform()
              ..origin.setValues(x,y,z)
            ),
            bullet.BoxShape(math.Vector3(w/2,h/2,d/2))
          )
        );
        bodys.add(sbody);
        world.addRigidBody(sbody);
        meshs.add(three.Mesh( geos['box'], mat ));
        meshs[i].scale.setValues( w, h, d );
      } 
      else if(t==3){
        three.Material mat = mats['cyl']!;
        mat.color = randColor;
        bullet.RigidBody sbody = bullet.RigidBody.constructor(
          bullet.RigidBodyConstructionInfo(
            1,
            bullet.DefaultMotionState(
              bullet.Transform()
              ..origin.setValues(x,y,z)
            ),
            bullet.CylinderShape(math.Vector3(w/2,h/2,d/2)),
          )
        );
        bodys.add(sbody);
        world.addRigidBody(sbody);
        meshs.add(three.Mesh( geos['cylinder'], mat));
        meshs[i].scale.setValues( w*0.5, h, w*0.5 );
      }

      meshs[i].castShadow = true;
      meshs[i].receiveShadow = true;

      threeJs.scene.add( meshs[i] );
    }
  }

  void updateBulletPhysics() {
    int ms = DateTime.now().microsecondsSinceEpoch;
    world.stepSimulation(ms / 1000000);

    double x, y, z;
    three.Mesh mesh; 
    bullet.RigidBody body;
    //print(bodys[0].getPosition());
    for(int i = 0; i < bodys.length;i++){
      body = bodys[i];
      mesh = meshs[i];

      if(body.isActive()){
        
        mesh.position.setFrom(body.worldTransform.origin.toVector3());
        mesh.quaternion.setFrom(math.Quaternion.fromRotation(body.worldTransform.basis).toQuaternion() );

        // change material
        if(mesh.material?.name == 'sbox') mesh.material = mats['box'];
        if(mesh.material?.name == 'ssph') mesh.material = mats['sph'];
        if(mesh.material?.name == 'scyl') mesh.material = mats['cyl']; 

        // reset position
        if(mesh.position.y<-100){
          x = -100 + m.Random().nextDouble() *200;
          z = -100 + m.Random().nextDouble() *200;
          y = 100 + m.Random().nextDouble() *1000;
          body.setLinearVelocity(math.Vector3(0,0,0));
          body.setAngularVelocity(math.Vector3(0,0,0));
          body.worldTransform.origin.setFrom(math.Vector3(x,y,z));
        }
      } 
      else {
        if(mesh.material?.name == 'box') mesh.material = mats['sbox'];
        if(mesh.material?.name == 'sph') mesh.material = mats['ssph'];
        if(mesh.material?.name == 'cyl') mesh.material = mats['scyl'];
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Stack(
        children: [
          threeJs.build(),
        ]
      )
    );
  }
}