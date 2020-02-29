

//  // USEFUL QUATERNIONS //  // 
// [w, x, y, z] = [cos(a/2), sin(a/2) * nx, sin(a/2)* ny, sin(a/2) * nz]
// [x]       [y]       [z]       [w]      //** description */
// [0]       [0]       [0]       [1]      //** identity */
// [1]       [0]       [0]       [0]      //** 180 around X : pitch */
// [0]       [1]       [0]       [0]      //** 180 around Y : yaw   */
// [0]       [0]       [1]       [0]      //** 180 around Z : roll  */
// [sq(.5)]  [0]       [0]       [sq(.5)] //** +90 around X : CW    */
// [0]       [sq(.5)]  [0]       [sq(.5)] //** +90 around Y : CW    */
// [0]       [0]       [sq(.5)]  [sq(.5)] //** +90 around Z : CW    */
// [-sq(.5)] [0]       [0]       [sq(.5)] //** -90 around X : CCW   */
// [0]       [-sq(.5)] [0]       [sq(.5)] //** -90 around Y : CCW   */
// [0]       [0]       [-sq(.5)] [sq(.5)] //** -90 around Z : CCW   */


// position [ x, y, z ] -> p = 0 + ix + jy + kz
// orientation [ heading    q = cos(a/2) + ix sin(a/2)
//                attitude               + jy sin(a/2)
//                   bank ]              + kz sin(a/2)
// rotation [ heading     q = cos(a/2) + ix sin(a/2)
//             attitude                + jy sin(a/2)
//                bank ]               + kz sin(a/2)
// rotate point      4D ->  q * p * conj(q)
// combine rotations 4D -> q1 * q2 
//
//  Pi -> vector before transform | Po -> vector after transform | q -> quaternion rep trans | conj() -> conjugate
//
// rotation ( around origin )               Po = q * Pi * conj( q )
// reflection (in plane thorough origin )   Po = q * Pi * q
// para component of plane                  Po = 1/2 ( Pi + q * Pi * q )
// perp component of plane                  Po = 1/2 ( Pi -q * Pi * q )
// scaling                                  Po = scalar * Pi (or comb w/rot or ref)
// translation                              Po = q + Pi
//
//  local? !reverse -> q1 then q2 ? q1 * q2 -> global? reverse -> q2 * q1




//  //************************************************************************************************// VARIABLES  //  //
//  //************************************************************************************************//
//  //************************************************************************************************//

const UNITVECTOR_X = new THREE.Vector3(1,0,0);
const UNITVECTOR_Y = new THREE.Vector3(0,1,0);
const UNITVECTOR_Z = new THREE.Vector3(0,0,1);

let log = document.getElementById( "log" );
let state_div = document.getElementById( "state" );
let msgs = [];


let container;
let camera, scene, renderer, controls, user, L, R, LH, LR; 

let tmp, ret;
let dust;
let paint = false;

let crosshair, 
    raycaster, 
    handRay, 
    paintRayL,
    paintRayR,
    intersected, 
    leftInter,
    rightInter,
    intersections = [],
    intersectionsLeft = [],
    intersectionsRight = [];

let tempMatrix = new THREE.Matrix4();
let rotationMatrix = new THREE.Matrix4();
let leftWristMatrix = new THREE.Matrix4();
let rightWristMatrix = new THREE.Matrix4();
let leftWristQuat = new THREE.Quaternion();
let rightWristQuat = new THREE.Quaternion();

let rot90 = new THREE.Quaternion();
let rot180 = new THREE.Quaternion();
let rotZ180 = new THREE.Quaternion();
let rotY90 = new THREE.Quaternion();

let tempQuaternion = new THREE.Quaternion();
let targetRotation = new THREE.Quaternion();
let fromHere = new THREE.Quaternion();
let toThere = new THREE.Quaternion();

let gloves, room, floor;
let planeX, planeY, planeZ;
let up, down, left, right;

let leftHand,
    leftHandControl, 
    rightHand,
    rightHandControl;

let leftWrist,
    leftJoints = [],
    rightWrist,
    rightJoints = [];

let geometries = [
  new THREE.ConeGeometry( 0.01, 0.05, 32 ), // [0] boids //0.01 0.02
  new THREE.TorusGeometry( 0.03, 0.01, 16, 100 ), // [1] wrist
  new THREE.BoxGeometry( 0.06, 0.01, 0.07 ), // [2] palm
  new THREE.BoxGeometry( 0.01, 0.01, 0.01 ), // [3] joints
  new THREE.TorusGeometry( 0.01, 0.001, 16, 100 ), // [4] fingertips
  new THREE.RingBufferGeometry( 0.02, 0.04, 32 ), // [5] crosshair
  //new THREE.BoxLineGeometry( 10, 10, 10, 10, 10, 10 ), // [6] room
  new THREE.PlaneBufferGeometry( 10, 10 ), // [7] floor
  new THREE.BufferGeometry().setFromPoints( [ new THREE.Vector3( 0, 0, 0 ), new THREE.Vector3( 0, 0, - 1 ) ] ) // [8] raycaster
];

let materials = [
  // [0] boids
  new THREE.MeshLambertMaterial( {
    color: 0x9a799c//0x191919 //0x9a799c //0x101010
  } ),
  // [1] wrist, palm, joints
  new THREE.MeshStandardMaterial( {
    color: 0x883322 
  } ),
  // [2] fingertips 
  new THREE.MeshToonMaterial ( {
    color: 0x166d99,
    transparent: true,
    opacity: 0
  } ),
  // [3] crosshair
  new THREE.MeshBasicMaterial( {
    color: 0xffffff,
    opacity: 0, //1
    transparent: true
  } ),
  // [4] room 
  new THREE.LineBasicMaterial( { 
    color: 0x808080
  } ),
  // [5] floor 
  new THREE.MeshStandardMaterial( {
    color: 0xffc30a,
    roughness: 1.0,
    metalness: 0.5
  } ),
  // [6] new floor
  new THREE.MeshLambertMaterial( {
    color: 0x101010//0x191919 //0x9a799c //0x101010
  } ),
  // [7] fingertips 
  new THREE.MeshToonMaterial ( {
    color: 0x160099
  } ),
  // [8] fingertips 
  new THREE.MeshToonMaterial ( {
    color: 0x166d00
  } ),
  // [9] fingertips 
  new THREE.MeshToonMaterial ( {
    color: 0x006d99,
    transparent: true,
    opacity: 0
  } ),
];

let state = null;

let sock;

//** 3D BOIDS */
let boids = new THREE.Group();
//let Creature;
let boundary = 7;
let clock = new THREE.Clock();
let flocks;
let axis = new THREE.Vector3();
let radians;
let Creatures = [];
let wind = new THREE.Vector3(0.05,0.0,0.0);
let gravity = new THREE.Vector3(0.0,0.1,0);

// VR STUFF
let isInVR = false;
let vrDisplay, frameData;
let rightEye, leftEye;

// STAR STUFF
let starTexture = new THREE.TextureLoader().load( "sparkle_cut.png" );
let stars = [];
let starGroup = new THREE.Group();
let lightness = 0;
let rotSpeed = 0.01;


//  //************************************************************************************************// UPDATE WORLD FUNCTIONS //  //
//  //************************************************************************************************//
//  //************************************************************************************************//

function getRandom(Min, Max) {
  let min = Min;
  let max = Max;
  let num = Math.floor(Math.random()*max) + min; // this will get a number between min and max;
  num *= Math.floor(Math.random()*2) == 1 ? 1 : -1; // this will add minus sign in 50% of cases
  return num;
}

//  //************************************************************************************************// FUNCTION CALLS  //  //
//  //************************************************************************************************//
//  //************************************************************************************************//

initialize();
animate();


//  //************************************************************************************************// SERVER CONNECT  //  //
//  //************************************************************************************************//
//  //************************************************************************************************//

//  //*************************************** // WRITE // ********************************************//

function write( ...args ) {

	if( msgs.length > 15 ) {

		msgs.shift();
  
  }

	let msg = args.join( ", " );
	msgs.push( msg );
  let fMsg = msgs.join( "\n" );

	log.innerText = "";
  log.innerText +=  "Log: \n " + fMsg;
  
  console.log( msg );
  
}
//  //*********************************** // CONNECT_TO_SERVER // *************************************//
function connect_to_server( opt, log ) {

	let self = {
    transport: opt.transport || "ws",
		hostname: opt.hostname || window.location.hostname,
		port: opt.port || window.location.port,
		protocols: opt.protocols || [],
		reconnect_period: 1000,
		reload_on_disconnect: true,
		socket: null,
  };
  
  self.addr = self.transport + '://' + self.hostname + ':' + self.port;
	
	let connect = function() {
  
    self.socket = new WebSocket( self.addr, self.protocols );
		self.socket.binaryType = 'arraybuffer';
    //self.socket.onerror = self.onerror;
    
		self.socket.onopen = function() {

			log( "websocket connected to " + self.addr );
			// ...
  
    }
  
    self.socket.onmessage = function( e ) { 
  
      if ( e.data instanceof ArrayBuffer ) {
  
        // if (onbuffer) {
				// 	//onbuffer(e.data, e.data.byteLength);
				// } else {

        log( "ws received arraybuffer of " + e.data.byteLength + " bytes" )

        //}

      } else {
  
        let msg = e.data;
				let obj
  
        try {
  
          obj = JSON.parse( msg );
  
        } catch( e ) {}
  
        if ( obj.cmd == "newData" ) {
  
          state = obj.state;
  
       } else if (obj.cmd == "trackingData") {

      /*
        ws received, {
          "cmd":"trackingData","state":
          {
            "hmd":
            {
              "pos":{"0":-0.31410789489746094,"1":1.6376476287841797,"2":0.4556894302368164},
              "quat":{"0":0.13929444551467896,"1":0.2569557726383209,"2":0.034140702337026596,"3":0.9557223320007324}
            },
            "trackers":
            [{
              "pos":{"0":0.9590294361114502,"1":1.911466121673584,"2":0.5492393970489502},
              "quat":{"0":-0.1990630030632019,"1":0.6774951219558716,"2":0.6710811257362366,"3":0.22588586807250977}
            },
            {
              "pos":{"0":1.086222529411316,"1":1.8866705894470215,"2":0.4619896411895752},
              "quat":{"0":-0.6431819200515747,"1":-0.2571682035923004,"2":-0.28171005845069885,"3":0.6639434695243835}
            }]
          }
         }
      */
          let lh = obj.state.trackers[0];
          let rh = obj.state.trackers[1];

         // log(lh.pos[1])

          // leftWrist.position.fromArray(lh.pos);
          // rightWrist.position.fromArray(rh.pos);

          // // apply 90 rot 
          // // [-/+sq(.5)]  [0]       [0]       [sq(.5)] //** -/+90 around X : CW    */
         
          // //rot90.set(Math.sqrt(0.5), 0, 0, Math.sqrt(0.5) );
          // rot90.setFromAxisAngle( new THREE.Vector3( 1, 0, 0 ), -Math.PI / 2 );

          // rotZ180.set( 0, 0, 1, 0 );
          // //rotZ180.setFromAxisAngle( new THREE.Vector3( 0, 0, 1 ), Math.PI );

          // rot180.set( 1, 0, 0, 0 );
          // //rot180.setFromAxisAngle( new THREE.Vector3( 1, 0, 0 ), Math.PI );
          
          // leftWrist.quaternion.fromArray(lh.quat);
          // rightWrist.quaternion.fromArray(rh.quat);
          
          // //leftWrist.quaternion.multiplyQuaternions(leftWrist.quaternion, rotZ180);
          // leftWrist.quaternion.multiplyQuaternions(leftWrist.quaternion, rot90);

          // rot180.set( 0, 1, 0, 0 );
          // //rot90.setFromAxisAngle( new THREE.Vector3( 1, 0, 0 ), -Math.PI/2 );

          // //rightWrist.quaternion.multiplyQuaternions(rightWrist.quaternion, rot180);
          // rightWrist.quaternion.multiplyQuaternions(rightWrist.quaternion, rot90);

          // // // check interactions
          // // leftHandControl = lh;
          // // rightHandControl = rh;

                  
          // // //  ** add controllers | hands | trackers */
          // // leftHandControl = renderer.vr.getController( 0 );
          // // leftHandControl.addEventListener( 'selectstart', onSelectStart );
          // // leftHandControl.addEventListener( 'selectend', onSelectEnd );
          // // //scene.add( leftHandControl );

          // // rightHandControl = renderer.vr.getController( 1 );
          // // rightHandControl.addEventListener( 'selectstart', onSelectStart );
          // // rightHandControl.addEventListener( 'selectend', onSelectEnd );
          // // //scene.add( rightHandControl );

          // // let line = new THREE.Line( geometries[8], materials[3] );
          // // //line.name = 'handRay';
          // // line.scale.z = 1;

          // // leftHandControl.add( line.clone() ); // leftWrist
          // // leftHandControl.name = 'leftHandRay';
          // // rightHandControl.add( line.clone() ); // rightWrist
          // // rightHandControl.name = 'rightHandRay';

          // //leftWrist.quaternion.multiplyQuaternions(leftWrist.quaternion, rot180);
          // //rightWrist.quaternion.multiplyQuaternions(rightWrist.quaternion, rot180);

          // // [0]       [0]      [1]       [0]      //** 180 around Z : roll  */
          // // rot180.set( 1, 0, 0, 0 );
          // // rot180.setFromAxisAngle( new THREE.Vector3( 1, 0, 0 ), Math.PI );



          // // rotY90.set( 0, Math.sqrt(0.5), 0, Math.sqrt(0.5) );
          // // rotY90.setFromAxisAngle( new THREE.Vector3( 0, 1, 0 ), Math.PI / 2 );
          
				} else {
          
          log( "ws received", msg );
  
        }
			} 
		}
  
    self.socket.onclose = function( e ) {
  
      self.socket = null;

			setTimeout( function() {
  
        if ( self.reload_on_disconnect ) {
  
          window.location.reload( true );
  
        } else {
  
          log( "websocket reconnecting" );

					connect();
  
        }
			}, self.reconnect_period );		
  
      //if (onclose) onclose(e);
			log( "websocket disconnected from " + self.addr );
  
    }

		self.send = function( msg ) {
  
      if ( !self.socket ) { console.warn( "socket not yet connected" ); return; }
			if ( self.socket.readyState != 1 ) { console.warn( "socket not yet ready" ); return; }
			if ( typeof msg !== "string" ) msg = JSON.stringify( msg );
  
      self.socket.send( msg );
  
    }
	}

	connect();

	return self;
}


//  //*********************************** // CONNECT_TO_HAPTICS // *************************************//


//  //************************************************************************************************// INITIALIZATION  //  //  
//  //************************************************************************************************//
//  //************************************************************************************************//

function initialize() {
  
  //** setup */
  container = document.createElement( 'div' );
  document.body.appendChild( container );

  let info = document.createElement( 'div' );
  info.style.position = 'absolute';
  info.style.top = '10px';
  info.style.width = '100%';
  info.style.textAlign = 'center';
  info.innerHTML = 'STUPID HACKATHON 2020';
  container.appendChild( info );

  //** add scene */
  scene = new THREE.Scene();
  scene.background = new THREE.Color( 0x808080 );
  //scene.fog	= new THREE.FogExp2( 0xdde0f0, 0.0025 );
  
  for (let i = 0; i < 300; i++) {
    let geometry = new THREE.SphereGeometry( 0.05, 8, 6 );
    let material = new THREE.MeshBasicMaterial( { map: starTexture } );
    let star = new THREE.Mesh( geometry, material );
    // let y = getRandom();
    // y *= y;
    // y = Math.sqrt( y );
    star.position.set( getRandom(0.1,10), getRandom(0.1,10), getRandom(0.1,10) );

    star.material.side = THREE.DoubleSide;
    stars.push( star );
  }

  for (let i = 0; i < 100; i++) { //100
    let geometry = new THREE.SphereGeometry( 0.2, 8, 6 );
    let material = new THREE.MeshBasicMaterial( { map: starTexture } );
    let star = new THREE.Mesh( geometry, material );
    // let y = getRandom();
    // y *= y;
    // y = Math.sqrt( y );
    star.position.set( getRandom(0.1,13), getRandom(0.1,13), getRandom(0.1,13) );

    star.material.side = THREE.DoubleSide;
    stars.push( star );
  }
  
  for (let i = 0; i < 25; i++) { //25
    let geometry = new THREE.SphereGeometry( 0.7, 8, 6 );
    let material = new THREE.MeshBasicMaterial( { map: starTexture } );
    let star = new THREE.Mesh( geometry, material );
    // let y = getRandom();
    // y *= y;
    // y = Math.sqrt( y );
    star.position.set( getRandom(0.1,15), getRandom(0.1,15), getRandom(0.1,15) );
    
    star.material.side = THREE.DoubleSide;
    stars.push( star );
  }

  for (let i = 0; i < 5; i++) { //5
    let geometry = new THREE.SphereGeometry( 1, 8, 6 );
    let material = new THREE.MeshBasicMaterial( { map: starTexture } );
    let star = new THREE.Mesh( geometry, material );
    // let y = getRandom();
    // y *= y;
    // y = Math.sqrt( y );
    star.position.set( getRandom(0.1,15), getRandom(0.1,15), getRandom(0.1,15) );
    
    star.material.side = THREE.DoubleSide;
    stars.push( star );
  }

  room = new THREE.Group();
  scene.add( room );
  tmp = stars.slice(stars);
  ret = [];
  for (let j = 0; j < stars.length; j++) {
    let index = Math.floor(Math.random() * tmp.length);
    let removed = tmp.splice(index, 1);
    // Since we are only removing one element
    ret.push(removed[0]);
    ret[j].name = "s" + j;
    starGroup.add( ret[j] );
  }

  scene.add(starGroup);
  starGroup.name = "starGroup";

  //gloves = new THREE.Group();
  //scene.add( gloves );

  
  user = new THREE.Group();
  user.name = "user"
  scene.add( user ); //room

  //gloves = new THREE.Group();
  //scene.add( gloves );

  //** add camera */
  const fov = 75;
  const aspect = window.innerWidth / window.innerHeight; // 2;  // the canvas default
  const near = 0.04; //0.1;
  const far = 20; //10 //50;
  camera = new THREE.PerspectiveCamera(fov, aspect, near, far);

  //** camera crosshairs | for intersections and to orientate sightline */
  crosshair = new THREE.Mesh( geometries[5], materials[3] );
  //camera.add( crosshair );
  crosshair.position.z = - 1; //** keep crosshair slightly infront of you at all times */
  //user.add( camera );
  scene.add( camera );
  //user.add( camera );

  // controls = new THREE.OrbitControls(camera);
  // camera.lookAt(0,  - .5, 0);

  let vec = new THREE.Vector3( 0, 0, -1 );
 // vec.applyQuaternion( camera.quaternion );
 // crosshair.position.copy( vec );
  
  //** add lighting */
  scene.add( new THREE.HemisphereLight( 0x808080, 0x606060 ) );
  scene.add( new THREE.AmbientLight( 0x404040 ) ); //** soft white light */
  let light = new THREE.DirectionalLight( 0xffffff );
  light.position.set( 1, 1, 1 ).normalize();
  light.castShadow = true;
  // light.shadow.camera.top = 6
  // light.shadow.camera.bottom = -6;
  // light.shadow.camera.right = 6;
  // light.shadow.camera.left = -6;
  // light.shadow.mapSize.set( 4096, 4096 );
  scene.add( light );


  //** add room */
  //room = new THREE.LineSegments( geometries[6], materials[4] );
  room.name = "room"
  
  
  room.position.set( 0, 0, 0 );
  
  
  //scene.add( room );
  
  //** add floor */
  //floor = new THREE.Mesh( geometries[7], materials[5] );
  floor = new THREE.Mesh( geometries[7],  materials[6] )
  floor.rotation.x = - Math.PI / 2;
  floor.name = "floor"
  floor.receiveShadow = true;
  scene.add( new THREE.PointLight( 0xff0040, 2, 50 ) );
  //scene.add(floor)
  
  //** add ray | used for casting lines from head + controllers to objects | used for intersecting */
  raycaster = new THREE.Raycaster();
  
  //** add rendered requirements */
  renderer = new THREE.WebGLRenderer( { antialias: true } );
  renderer.setPixelRatio( window.devicePixelRatio );
  renderer.setSize( window.innerWidth, window.innerHeight );
  renderer.gammaInput = true;
  renderer.gammaOutput = true;
  renderer.shadowMap.enabled = true;

  //** include VR content */
  renderer.vr.enabled = true;
  
  container.appendChild( renderer.domElement );
  document.body.appendChild( WEBVR.createButton( renderer ) );
  
  //** wrist */
  // leftWrist = new THREE.Mesh( geometries[1], materials[2] );
  // rightWrist = new THREE.Mesh( geometries[1], materials[2] );
  
  // leftWrist.position.set( 0, 1.3, + 0.1 ); // 0, 1.5, 0
  // rightWrist.position.set( 0, 1.3, - 0.1 ); // 0.5, 1.5, -1

  // leftWrist.add( new THREE.AxesHelper( 0.05 ) );
  // rightWrist.add( new THREE.AxesHelper( 0.05 ) );

  // leftWrist.name = "leftWrist"
  // rightWrist.name = "rightWrist"

  // user.add( leftWrist );
  // user.add( rightWrist );

  //** add controllers | hands | trackers */
  leftHandControl = renderer.vr.getController( 0 );
  leftHandControl.addEventListener( 'selectstart', onSelectStart );
  leftHandControl.addEventListener( 'selectend', onSelectEnd );
  scene.add( leftHandControl );

  rightHandControl = renderer.vr.getController( 1 );
  rightHandControl.addEventListener( 'selectstart', onSelectStart );
  rightHandControl.addEventListener( 'selectend', onSelectEnd );
  scene.add( rightHandControl );

  let line = new THREE.Line( geometries[8] );
  line.name = 'handRay';
  line.scale.z = 1;

  leftHandControl.add( line.clone() );
  rightHandControl.add( line.clone() );

  // leftHand.add( new THREE.AxesHelper( 0.05 ) );  
  // palm.add( new THREE.AxesHelper( 0.05 ) );
  scene.add( new THREE.AxesHelper( 1 ) );

  //** add event listeners  */
  // window.addEventListener( 'vrdisplaypointerrestricted', onPointerRestricted, false );
  // window.addEventListener( 'vrdisplaypointerunrestricted', onPointerUnrestricted, false );
  window.addEventListener( 'resize', onWindowResize, false );

  // //** 3D BOIDS */
  // flock = new THREE.Group();
  // room.add(flock);

  try {
    sock = connect_to_server( {}, write );
  } catch ( e ) {
    console.error( e );
  }

}



//  //************************************************************************************************// INTERSECTIONS  //  // 
//  //************************************************************************************************//
//  //************************************************************************************************//

//  //*************************************** // CLEAN // ********************************************//
function cleanIntersected() {

  while ( intersections.length ) {

    let object = intersections.pop();
    object.material.emissive.r = 0;

  }

}


//  //********************************** // SELECT START // ******************************************//
function onSelectStart( event ) {

  let selectStart = event.target;
  let intersections = getIntersections( selectStart );

  if ( intersections.length > 0 ) {

    let intersection = intersections[ 0 ];

    tempMatrix.getInverse( selectStart.matrixWorld );

    let object = intersection.object;
    object.matrix.premultiply( tempMatrix );
    object.matrix.decompose( object.position, object.quaternion, object.scale );
    //object.material.emissive.b = 1;
    selectStart.add( object );

    selectStart.userData.selected = object;

  }

}

//  //************************************ // SELECT END // ******************************************//
function onSelectEnd( event ) {

  let selectEnd = event.target;

  if ( selectEnd.userData.selected !== undefined ) {

    let object = selectEnd.userData.selected;
    object.matrix.premultiply( selectEnd.matrixWorld );
    object.matrix.decompose( object.position, object.quaternion, object.scale );
    //object.material.emissive.b = 0;
    scene.add( object );

    selectEnd.userData.selected = undefined;

  }


}

//  //******************************** // GET INTERSECTIONS // ***************************************//
function getIntersections( event ) {

  tempMatrix.identity().extractRotation( event.matrixWorld );

  raycaster.ray.origin.setFromMatrixPosition( event.matrixWorld );
  raycaster.ray.direction.set( 0, 0, -1 ).applyMatrix4( tempMatrix );

  return raycaster.intersectObjects( user.children );

}

//  //********************************* // INTERSECT OBJECT // ***************************************//
function intersectObjects( handAction ) {

  //* Do not highlight when already selected */

  if ( handAction.userData.selected !== undefined ) return;

  let handRay = handAction.getObjectByName( 'handRay' );
  let intersections = getIntersections( handAction );

  if ( intersections.length > 0 ) {

    let intersection = intersections[ 0 ];

    let object = intersection.object;
    object.material.emissive.r = 1;
    // object.rotation.y += 0.1;
    intersections.push( object );

    handRay.scale.z = intersection.distance;

  } else {

    handRay.scale.z = 5;

  }

}

//  //********************************** // INTERSECT HEAD // ****************************************//
function intersectHead() {
  
  raycaster.setFromCamera( { x: 0, y: 0 }, camera );
  let intersects = raycaster.intersectObjects( user.children );
  
  if ( intersects.length > 0 ) {
    
    if ( intersected != intersects[ 0 ].object ) {
      
      if ( intersected ) intersected.material.emissive.setHex( intersected.currentHex );
      
      intersected = intersects[ 0 ].object;
      intersected.currentHex = intersected.material.emissive.getHex();
      intersected.material.emissive.setHex( 0xff0000 );
      // intersected.rotation.y += 0.5;
      // intersected.position.z -= 0.1;

      if (intersected) {
        try {
          sock.send( "sendHaptics" );
        } catch( e ) {
          write( e );
        }
      }
    } else {
    
    if ( intersected ) intersected.material.emissive.setHex( intersected.currentHex );

    intersected = undefined;
    
    }
  }
}

//  //************************************ // CHECK ROOM // ******************************************//
function checkRoom() {
  
  for ( let i = 0; i < room.children.length; i ++ ) {
    
    let canTouch = room.children[ i ];

    if ( canTouch.position.x < - 2.5 || canTouch.position.x > 2.5 ) {
      
      canTouch.position.x = THREE.Math.clamp( canTouch.position.x, - 2.5, 2.5 );
      
    }  //** canTouch position x */
    
    if ( canTouch.position.y < - 2.5 || canTouch.position.y > 3 ) {
      
      canTouch.position.y = THREE.Math.clamp( canTouch.position.y, - 3, 3 );
      
    }  //** canTouch position y */
    
    if ( canTouch.position.z < - 2.5 || canTouch.position.z > 2.5 ) {
      
      canTouch.position.z = THREE.Math.clamp( canTouch.position.z, - 2.5, 2.5 );
      
    } //** canTouch position z */
    
  } 
}

//  //************************************ // CREATURE // ******************************************//
function buildCreature() {
  this.findUser = function(bump) {
    let d = this.position.distanceTo(bump.position);
    if ((d > 0) && (d < 1)) {
        try {
          sock.send( "sendHaptics_back" );
        } catch( e ) {
          write( e )
        }
     }

  }
}

//  //************************************************************************************************/ 
//  //************************************************************************************************/  UPDATE VR WORLD  //  //
//  //************************************************************************************************/ 

function onWindowResize() {

  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();

  renderer.setSize( window.innerWidth, window.innerHeight );

}

function animate() {

  renderer.setAnimationLoop( render );

}

function render() {
  
  //controls.update();

  try {
    sock.send( "getData" );
  } catch( e ) {
    write( e )
  }

  if ( !state ) return;

  //** manage intersections */
  cleanIntersected();
  getIntersections();
  intersectObjects( leftHandControl );
  intersectObjects( rightHandControl );
  intersectHead();
  //checkRoom();
  
  //scene.updateMatrixWorld();


  for (let k = 0; k < ret.length; k++) {
    let star = ret[k];
    star.rotation.x += 0.01*k/100;
    //star.rotation.y += 0.01/k;
    star.rotation.z += 0.01*k/100;
    lightness > 100 ? lightness = 0 : lightness+=0.4; //++
    //let material = new THREE.MeshLambertMaterial( { color: new THREE.Color("hsl(" + H + ", 100%, 80%)" ), transparency: true, opacity: 0.4 } );
    //   planeMaterial.color.setHSL(object.userData.H,object.userData.S,object.userData.L);
    //star.material.color = new THREE.Color("hsl( 255, 100%, " + lightness + "%)");
    star.material.color.setHSL(255, 100, lightness);
  } 

  let invertStage = new THREE.Matrix4()
  let temp = new THREE.Matrix4();
  let vrDisplay = renderer.vr.getDevice()
  if (vrDisplay) {
    temp.fromArray( vrDisplay.stageParameters.sittingToStandingTransform )
    //invertStage.getInverse( temp, true );
    //temp.getInverse( temp )
    //console.log(temp)
    //user.position.set(invertStage[12])
    user.position.fromArray(temp.elements, 12)
   // scene.position.fromArray(temp.elements, 12);
  } 

  renderer.render( scene, camera );

}