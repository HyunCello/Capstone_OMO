// Initialize Firebase
// Your web app's Firebase configuration
var firebaseConfig = {
  apiKey: "AIzaSyDo3ePhiSi8uHkQHAUPlSU-Em2APipaXUE",
  authDomain: "omo-mobile.firebaseapp.com",
  databaseURL: "https://omo-mobile.firebaseio.com",
  projectId: "omo-mobile",
  storageBucket: "omo-mobile.appspot.com",
  messagingSenderId: "126047002730",
  appId: "1:126047002730:web:ebb27bd8a261f831856a13",
  measurementId: "G-ZECGZQ6K3H",
};
// Initialize Firebase
firebase.initializeApp(firebaseConfig);
/*
    // 구글 인증 기능 추가
    var provider = new firebase.auth.GoogleAuthProvider();
  
    // 인증하기
    firebase.auth().signInWithPopup(provider).then(function (result) {
      // This gives you a Google Access Token. You can use it to access the Google API.
      var token = result.credential.accessToken;
      // The signed-in user info.
      var user = result.user;
  
      console.log(user)		// 인증 후 어떤 데이터를 받아오는지 확인해보기 위함.
    });
  */
firebase.analytics();

var database = firebase.database(); // [데이터베이스] SDK 초기화

////////읽기 예제////////
/*
  var dbTestRef = database.ref('test/')
  dbTestRef.on('child_added', function (data) {
    console.log(data.val())
  })
*/

////수정 예제/////
/*
  var dbTestRef = database.ref('test/')
  dbTestRef.on('child_added', function (data) {
    console.log(data.val(), 'key: ', data.key)
  })
*/

////////삭제 예제//////////
dbTestRef3 = database.ref("test/-L4eNay35sc0db4oigfO").remove();
/*
    var newOrderKey = firebase.database().ref().child('orders').push().key;

    var orderData = {
      brdno: newOrderKey,
      brdwriter:"홍길동",
      brdmenu:"주문 종류",
      brdnumber:"메뉴 개수",
      brddate: Date.now()
    };

    var updates = {};
    updates['/orders/' + newOrderKey] = orderData;

    firebase.database().ref().update(updates);
      */

function addmenu(menu, location) {
  /////////// push = db에 추가 //////////////////
  database.ref("test/").push({ name: menu, location: location });
  /////////// set = 다른거 없애고 그냥 만듬 ///////////////
  alert(menu + " 주문되었습니다");
}

function readmenu() {
  var dbTestRef = database.ref("test/");
  dbTestRef.on("child_added", function (data) {
    console.log(data.val().name, "key: ", data.key);
  });
}

function editmenu() {
  var dbTestRef = database.ref("test/");
  dbTestRef.on("child_added", function (data) {
    console.log(data.val(), "key: ", data.key);
  });
  // dbTestRef2 = database.ref('test/' + [key값]).update([수정할 값])
  dbTestRef2 = database.ref("test/-M65Ux2lgQbYsUWBE9JR").update({
    intro: "인삿말 수정",
  });
}

function deletemenu(key) {
  // var keys = [];
  // var dbTestRef = database.ref("test/");
  // setTimeout(
  //   dbTestRef.on("child_added", function (data) {
  //     keys.push(data.key);
  //   }),
  //   1000
  // );
  var dbTestRef3 = database.ref("test/" + key).remove();
  console.log(key);
  alert("처리되었습니다");
  location.href = "seller.html"
}
