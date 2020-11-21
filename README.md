# speech_recognition_pkg
* USBマイクで音声を録音し、音声を認識するノード（speech_recogniton_class.py）<br>
* 音声認識の結果を発話するノード（play_voice_class.py）<br>
* 音声認識の結果を形態素解析し，結果を出力するノード（keitaiso_class.py）<br>

をまとめたROSのパッケージです．<br>
音声録音には**PyAudio**，音声認識には**SpeechRecognition**，音声合成には**Google TTS**，形態素解析には**Mecab**を利用しています．<br>
**Ubuntu 16.04 LTS**の**ROS Kinetic**，**Python 2.7**で動作確認済みです．

## 起動方法
`roslaunch speech_recognition_pkg speech_recognition_pkg_class.launch`で<br>
* speech_recogniton_class.py（音声録音、音声認識）<br>
* play_voice_class.py（音声合成、音声再生）<br>
* keitaiso_class.py（形態素解析）<br>

の3つのノードを一気に立ち上げます．<br>

## 補足事項
output.mp3にはGoogle TTSにより音声合成した音声データが入ります．<br>
ファイル名の「_class」は，classでコードを記述しているという意味です．ファイル名に「_class」がない場合は，classを使用せずにコードを記述しています．
