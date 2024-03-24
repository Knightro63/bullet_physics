import 'dart:async';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/foundation.dart';

import 'package:flutter_gl/flutter_gl.dart';
import 'package:bullet_physics/bullet_physics.dart' as bullet;
import 'package:three_dart/three_dart.dart' as three;
import 'package:three_dart/three_dart.dart' hide Texture, Color;
import 'package:three_dart_jsm/three_dart_jsm.dart';

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
  Quaternion toQuaternion(){
    return Quaternion(x,y,z,w);
  }
}
extension on math.Vector3{
  Vector3 toVector3(){
    return Vector3(x,y,z);
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
  FocusNode node = FocusNode();
  // gl values
  //late Object3D object;
  bool animationReady = false;
  late FlutterGlPlugin three3dRender;
  WebGLRenderTarget? renderTarget;
  WebGLRenderer? renderer;
  late OrbitControls controls;
  int? fboId;
  late double width;
  late double height;
  Size? screenSize;
  late Scene scene;
  late Camera camera;
  double dpr = 1.0;
  bool verbose = false;
  bool disposed = false;
  final GlobalKey<DomLikeListenableState> _globalKey = GlobalKey<DomLikeListenableState>();
  dynamic sourceTexture;

  List<Mesh> meshs = [];
  List<Mesh> grounds = [];

  Map<String,BufferGeometry> geos = {};
  Map<String,three.Material> mats = {};

  //oimo var
  late bullet.DynamicsWorld world;
  late double lastCallTime;
  List<bullet.RigidBody> bodys = [];

  List<int> fps = [0,0,0,0];
  double toRad = 0.0174532925199432957;
  int type = 1;
  int max = 200;

  @override
  void initState() {
    super.initState();
  }
  @override
  void dispose() {
    disposed = true;
    three3dRender.dispose();
    super.dispose();
  }
  
  void initSize(BuildContext context) {
    if (screenSize != null) {
      return;
    }

    final mqd = MediaQuery.of(context);

    screenSize = mqd.size;
    dpr = mqd.devicePixelRatio;

    initPlatformState();
  }
  void animate() {
    if (!mounted || disposed) {
      return;
    }

    render();

    Future.delayed(const Duration(milliseconds: 1000~/60), () {
      updateCannonPhysics();
      animate();
    });
  }
  Future<void> initPage() async {
    
    scene = Scene();

    camera = PerspectiveCamera(60, width / height, 1, 10000);
    camera.position.set(0,160,400);

    controls = OrbitControls(camera, _globalKey);
    //controls.target.set(0,20,0);
    //controls.update();
    
    scene.add(AmbientLight( 0x3D4143 ) );
    DirectionalLight light = DirectionalLight( 0xffffff , 1.4);
    light.position.set( 300, 1000, 500 );
    light.target!.position.set( 0, 0, 0 );
    light.castShadow = true;

    int d = 300;
    light.shadow!.camera = OrthographicCamera( -d, d, d, -d,  500, 1600 );
    light.shadow!.bias = 0.0001;
    light.shadow!.mapSize.width = light.shadow!.mapSize.height = 1024;

    scene.add( light );

    // background
    BufferGeometry buffgeoBack = three.IcosahedronGeometry(3000,2);
    Mesh back = three.Mesh( 
      buffgeoBack, 
      three.MeshLambertMaterial()
    );
    scene.add( back );

    // geometrys
    geos['sphere'] = three.SphereGeometry(1,16,10);
    geos['box'] =  three.BoxGeometry(1,1,1);
    geos['cylinder'] = three.CylinderGeometry(1,1,1);
    
    // materials
    mats['sph']    = MeshPhongMaterial({'shininess': 10, 'name':'sph'});
    
    mats['box']    = MeshPhongMaterial({'shininess': 10, 'name':'box'});
    mats['cyl']    = MeshPhongMaterial({'shininess': 10, 'name':'cyl'});
    mats['ssph']   = MeshPhongMaterial({'shininess': 10, 'name':'ssph'});
    mats['sbox']   = MeshPhongMaterial({'shininess': 10, 'name':'sbox'});
    mats['scyl']   = MeshPhongMaterial({'shininess': 10, 'name':'scyl'});
    mats['ground'] = MeshPhongMaterial({'shininess': 10, 'color':0x3D4143, 'transparent':true, 'opacity':0.5});

    animationReady = true;
  }

  void addStaticBox(List<double> size,List<double> position,List<double> rotation) {
    Mesh mesh = three.Mesh( geos['box'], mats['ground'] );
    mesh.scale.set( size[0], size[1], size[2] );
    mesh.position.set( position[0], position[1], position[2] );
    mesh.rotation.set( rotation[0]*toRad, rotation[1]*toRad, rotation[2]*toRad );
    scene.add( mesh );
    grounds.add(mesh);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
  }

  void clearMesh(){
    for(int i = 0; i < meshs.length;i++){ 
      scene.remove(meshs[i]);
    }

    for(int i = 0; i < grounds.length;i++){ 
      scene.remove(grounds[ i ]);
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
		math.Vector3 worldAabbMin = math.Vector3(-10000, -10000, -10000);
		math.Vector3 worldAabbMax = math.Vector3(10000, 10000, 10000);
		int maxProxies = 1024;
		//bullet.AxisSweep3 broadphase = bullet.AxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

		bullet.SequentialImpulseConstraintSolver sol = bullet.SequentialImpulseConstraintSolver();
		bullet.ConstraintSolver solver = sol;
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
    // add ground
    world.addRigidBody(
      bullet.RigidBody.constructor(
        bullet.RigidBodyConstructionInfo(
          0,
          bullet.DefaultMotionState(
            bullet.Transform()
            ..origin.setValues(-180.0,20.0,0.0)
          ),
          bullet.BoxShape(math.Vector3(40.0/2, 40.0/2, 390.0/2)),
        )
      )
    );
    world.addRigidBody(
      bullet.RigidBody.constructor(
        bullet.RigidBodyConstructionInfo(
          0,
          bullet.DefaultMotionState(
            bullet.Transform()
            ..origin.setValues(180.0,20.0,0.0)
          ),
          bullet.BoxShape(math.Vector3(40.0/2, 40.0/2, 390.0/2)),
        )
      )
    );
    world.addRigidBody(
      bullet.RigidBody.constructor(
        bullet.RigidBodyConstructionInfo(
          0,
          bullet.DefaultMotionState(
            bullet.Transform()
            ..origin.setValues(0.0,-40.0,0.0)
          ),
          bullet.BoxShape(math.Vector3(400.0/2, 80.0/2, 400.0/2)),
        )
      )
    );

    addStaticBox([40, 40, 390], [-180,20,0], [0,0,0]);
    addStaticBox([40, 40, 390], [180,20,0], [0,0,0]);
    addStaticBox([400, 80, 400], [0,-40,0], [0,0,0]);

    //add object
    double x, y, z, w, h, d;
    int t;
    for(int i = 0; i < max;i++){
      if(type==4) {
        t = Math.floor(Math.random()*3)+1;
      }
      else {
        t = type;
      }
      x = -100 + Math.random()*200;
      z = -100 + Math.random()*200;
      y = 100 + Math.random()*1000;
      w = 10 + Math.random()*10;
      h = 10 + Math.random()*10;
      d = 10 + Math.random()*10;
      three.Color randColor = three.Color().setHex((Math.random() * 0xFFFFFF).toInt());

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
        meshs[i].scale.set( w*0.5, w*0.5, w*0.5 );
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
        meshs[i].scale.set( w, h, d );
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
        meshs[i].scale.set( w*0.5, h, w*0.5 );
      }

      meshs[i].castShadow = true;
      meshs[i].receiveShadow = true;

      scene.add( meshs[i] );
    }
  }

  void updateCannonPhysics() {
    int ms = DateTime.now().microsecondsSinceEpoch;
    world.stepSimulation(ms / 1000000);

    double x, y, z;
    Mesh mesh; 
    bullet.RigidBody body;
    //print(bodys[0].getPosition());
    for(int i = 0; i < bodys.length;i++){
      body = bodys[i];
      mesh = meshs[i];

      if(body.isActive()){
        
        mesh.position.copy(body.worldTransform.origin.toVector3());
        mesh.quaternion.copy(math.Quaternion.fromRotation(body.worldTransform.basis).toQuaternion() );

        // change material
        if(mesh.material.name == 'sbox') mesh.material = mats['box'];
        if(mesh.material.name == 'ssph') mesh.material = mats['sph'];
        if(mesh.material.name == 'scyl') mesh.material = mats['cyl']; 

        // reset position
        if(mesh.position.y<-100){
          x = -100 + Math.random()*200;
          z = -100 + Math.random()*200;
          y = 100 + Math.random()*1000;
          body.worldTransform.origin.setFrom(math.Vector3(x,y,z));
        }
      } 
      else {
        if(mesh.material.name == 'box') mesh.material = mats['sbox'];
        if(mesh.material.name == 'sph') mesh.material = mats['ssph'];
        if(mesh.material.name == 'cyl') mesh.material = mats['scyl'];
      }
    }
  }

  void render() {
    final _gl = three3dRender.gl;
    renderer!.render(scene, camera);
    _gl.flush();
    controls.update();
    if(!kIsWeb) {
      three3dRender.updateTexture(sourceTexture);
    }
  }
  void initRenderer() {
    Map<String, dynamic> _options = {
      "width": width,
      "height": height,
      "gl": three3dRender.gl,
      "antialias": true,
      "canvas": three3dRender.element,
    };

    if(!kIsWeb && Platform.isAndroid){
      _options['logarithmicDepthBuffer'] = true;
    }

    renderer = WebGLRenderer(_options);
    renderer!.setPixelRatio(dpr);
    renderer!.setSize(width, height, false);
    renderer!.shadowMap.enabled = true;
    renderer!.shadowMap.type = three.PCFShadowMap;
    //renderer!.outputEncoding = three.sRGBEncoding;

    if(!kIsWeb){
      WebGLRenderTargetOptions pars = WebGLRenderTargetOptions({"format": RGBAFormat,"samples": 8});
      renderTarget = WebGLRenderTarget((width * dpr).toInt(), (height * dpr).toInt(), pars);
      renderer!.setRenderTarget(renderTarget);
      sourceTexture = renderer!.getRenderTargetGLTexture(renderTarget!);
    }
    else{
      renderTarget = null;
    }
  }
  void initScene() async{
    await initPage();
    initRenderer();
    initBulletPhysics();
    animate();
  }
  Future<void> initPlatformState() async {
    width = screenSize!.width;
    height = screenSize!.height;

    three3dRender = FlutterGlPlugin();

    Map<String, dynamic> _options = {
      "antialias": true,
      "alpha": true,
      "width": width.toInt(),
      "height": height.toInt(),
      "dpr": dpr,
      'precision': 'highp'
    };
    await three3dRender.initialize(options: _options);

    setState(() {});

    // TODO web wait dom ok!!!
    Future.delayed(const Duration(milliseconds: 100), () async {
      await three3dRender.prepareContext();
      initScene();
    });
  }

  Widget threeDart() {
    return Builder(builder: (BuildContext context) {
      initSize(context);
      return Container(
        width: screenSize!.width,
        height: screenSize!.height,
        color: Theme.of(context).canvasColor,
        child: DomLikeListenable(
          key: _globalKey,
          builder: (BuildContext context) {
            FocusScope.of(context).requestFocus(node);
            return Container(
              width: width,
              height: height,
              color: Theme.of(context).canvasColor,
              child: Builder(builder: (BuildContext context) {
                if (kIsWeb) {
                  return three3dRender.isInitialized
                      ? HtmlElementView(
                          viewType:
                              three3dRender.textureId!.toString())
                      : Container();
                } else {
                  return three3dRender.isInitialized
                      ? Texture(textureId: three3dRender.textureId!)
                      : Container();
                }
              })
            );
          }
        ),
      );
    });
  }

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      height: double.infinity,
      width: double.infinity,
      child: Stack(
        children: [
          threeDart(),
        ],
      )
    );
  }
}