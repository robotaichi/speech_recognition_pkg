# speech_recognition_pkg
USBマイクで音声を録音すると，その音声を認識し，その結果を発話、形態素解析の結果を出力するROSのパッケージです．
音声録音にはPyAudio，音声認識にはSpeechRecognition，音声合成にはGoogle TTS，形態素解析にはMecabを利用しています．
Ubuntu 16.04 LTSのROS Kinetic，Python 2.7で動作確認済みです．
