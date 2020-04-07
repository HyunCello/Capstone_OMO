# Tensorflow Object Detection 예제

yolo를 썼는데 fps가 낮아서

tiny-yolo를 써보았지만 그마저도 fps가 너무 낮았다.

할 수 없이 찾은 것이 바로 이것 "Tensorflow Object Detection"이고

이 것은 ukayzm님이 만들어 놓으신 예제코드이다.

[https://ukayzm.github.io/python-object-detection-tensorflow/](https://ukayzm.github.io/python-object-detection-tensorflow/)

우선 이것을 하기 전에 tensorflow와 python이 깔려 있어야 한다.

### Dependencies

- Protobuf 3.0.0
- Python-tk
- Pillow 1.0
- lxml
- tf Slim (which is included in the "tensorflow/models/research/" checkout)
- Jupyter notebook
- Matplotlib
- Tensorflow (>=1.12.0)
- Cython
- contextlib2
- cocoapi

라이브러리 import 과정은 다음 사이트를 참고하면 된다.

[https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md)

    

위에 꺼 다 무시하고 이거부터 시작해도 될 거 같다...

[https://github.com/ukayzm/opencv/tree/master/object_detection_tensorflow](https://github.com/ukayzm/opencv/tree/master/object_detection_tensorflow)

    git clone https://github.com/ukayzm/opencv.git

github에서 다운 받는다.

    cd opencv

    cd object_detection_tensorflow

/home/사용자/opencv/object_detection_tensorflow 경로에 들어간다.

    python object_detector.py

실행파일 실행

    detector = ObjectDetector('ssd_mobilenet_v1_coco_2017_11_17')
    #detector = ObjectDetector('mask_rcnn_inception_v2_coco_2018_01_28')
    #detector = ObjectDetector('pet', label_file='data/pet_label_map.pbtxt')

이런 식으로 쓰고 싶은 모델을 설정하여 사용할 수 있다.

설정한 후 저장하여 object_detector.py를 실행하게 되면 파일 내에서 자동으로 다운받아 적용한다.

하지만 목표는 bounding box의 넓이를 통해 사람과의 거리를 조절하는 것이었기 때문에 일단 bounding box의 좌표값을 알아내는 것이 필요했다.

코드를 보니 numpy가 아니고 tensorflow 함수를 쓰는 코드여서인지 어떻게 해야 할지 감이 안왔다.

(사실 numpy도 잘 모르고 python도 모름;;)

하지만 방법을 알아내었는데 그것은 바로

    self.output_dict['detection_boxes'],
    self.output_dict['detection_classes'],
    self.output_dict['detection_scores'],
    
    instance_masks=self.output_dict.get('detection_masks')

이 친구들을 사용하는 것이었다.

detection_boxes: detect된 class의 좌표 정보 (ymin, xmin, ymax, xmax)

detection_classes: detect된 class번호를 나타냄 (ex. 1번은 사람, 5번은 비행기)

detection_scores: detect된 class의 accuracy를 나타냄

    output_dict = sess.run(self.tensor_dict,
                                     feed_dict={image_tensor: np.expand_dims(image_np, 0)})

output_dict라는 것을 쓰는 것이 핵심이었는데, tensorflow함수의 일종이었다.

run이라는 것도 마찬가지였다.(이거 때매 시간 많이 날림;;)

detect_objects 함수는 물체를 인식한 후 bounding box를 쳐주는 이 코드의 핵심 함수이다.

이 함수 안으로 들어가서,

vis_util.visualize_boxes_and_labels_on_image_array 함수 블록 바로 다음에 코드를 작성하였다.

검색하다보니 다행히 나같이 bounding box의 좌표를 찾고 싶어하는 글을 볼 수 있었고,

[https://github.com/tensorflow/models/issues/4682](https://github.com/tensorflow/models/issues/4682)

여기서 사람들이 올린 코드 중에 될 것 같은 것을 해봤더니.... 됫다..!

    	classes = self.output_dict['detection_classes']
            boxes = self.output_dict['detection_boxes']

일단 코드명을 줄이기 위해 이름을 바꾸었고,

    for i in range(len(boxes)):
                #print('classes: ', classes[i])
                #print('boxes: ', boxes[i])
                height = (boxes[0][2] - boxes[0][0]) * 480
                width= (boxes[0][3] - boxes[0][1]) * 640
                area= width * height
                #print("area: %0.1f" % area )
                mid_height = (boxes[0][2] + boxes[0][0])/2 * 480
                mid_width = (boxes[0][3] + boxes[0][1])/2 * 640
    
                # print('mid_width: ', mid_width)
                # print('mid_height: ', mid_height)
    
                if (classes == 1).any():
    
                    if (area > 130000) :
                        print('Person too close !!')
                        topic_msg = 'Stop'
    
                    elif (mid_width > 320) & (area > 80000) :
                        print('Turn left to avoid person')
                        topic_msg = 'Left'
    
                    elif (mid_width < 320) & (area > 80000) :
                        print('Turn right to avoid person')
                        topic_msg = 'Right'
    
                    else :
                        print("person detected & area: %0.1f" % area )
                        topic_msg = 'Keep going'

이런 식으로 코드를 작성하였다.

if문에 있는 topic_msg값은 ros에 활용하려고 쓴 것이니 무시해도 상관없다.

boxes의 경우 2차원 배열인지 3차원 배열인지 확실하지 않은데 (위의 참고 링크에서도 둘다 있었음)

3차원으로 썼더니 안되길래 2차원으로 썼더니 되었다.

아까 알려준대로

detection_boxes: detect된 class의 좌표 정보 (ymin, xmin, ymax, xmax)

의 정보를 참고하면

boxes[0][0]         :         ymin

boxes[0][1]         :         xmin

boxes[0][2]         :         ymax

boxes[0][3]         :         xmax

이다.

    for i in range(len(boxes)):
                #print('classes: ', classes[i])
                #print('boxes: ', boxes[i])

값을 확인하기 위해 주석을 빼고 프린트해보면 

classes의 경우 번호가, boxes의 경우 4차원 배열이 출력된다.

boxes의 경우 4차원 배열의 값을 살펴보면 다 소수인 것을 알 수 있는데,

아마 전체 x축과 전체 y축의 최대값을 1으로 설정한 것으로 추측할 수 있다.

따라서 실제로 사용할 때는 자신이 띠울 frame의 가로와 세로길이에 맞게 곱해서 사용해야 한다.

나 같은 경우는 이런 식으로 활용했다. (가로 640, 세로 480)

    	    height = (boxes[0][2] - boxes[0][0]) * 480
                width= (boxes[0][3] - boxes[0][1]) * 640
                area= width * height
                #print("area: %0.1f" % area )

조건문의 경우에는 classes를 이용하여 쓰면 되는 줄 알았는데 그대로 사용하니 에러가 떴다.

(대충 classes가 많이 뜨니 a.any() 이나 a.all()으로 사용하라는 에러)

 

    if (classes == 1).any():
    
                    if (area > 130000) :
                        print('Person too close !!')
                        topic_msg = 'Stop'
    
                    elif (mid_width > 320) & (area > 80000) :
                        print('Turn left to avoid person')
                        topic_msg = 'Left'
    
                    elif (mid_width < 320) & (area > 80000) :
                        print('Turn right to avoid person')
                        topic_msg = 'Right'
    
                    else :
                        print("person detected & area: %0.1f" % area )
                        topic_msg = 'Keep going'

그리하여 all()과 any()를 둘 다 써 본 결과 any를 썼을 때 정상적으로 작동하였다.

아마 any는 특별히 괄호()안의 것에 대해 동작하고, all은 괄호() 말고도 여러 것에 대해 동작하는 것 같다(?)
