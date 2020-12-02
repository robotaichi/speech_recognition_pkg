#!/usr/bin/env python
# -*- coding: utf-8 -*-
#上記2行は必須構文のため、コメント文だと思って削除しないこと
#Python2.7用プログラム

import rospy
#import os
#import time
#import datetime
#import sys
#import glob
#from pprint import pprint
import speech_recognition as sr
#import MeCab #形態素解析ライブラリ
import wave #wavファイルを扱うライブラリ
import pyaudio
from pydub import AudioSegment
from pydub.utils import make_chunks
from gtts import gTTS
from speech_recognition_pkg.msg import speech_recognition_message #メッセージファイルの読み込み（from パッケージ名.msg import 拡張子なしメッセージファイル名）



#変数設定
Language = 'ja-JP' #音声認識の対象言語を設定
#wavファイルの指定#
file = '/home/limlab/catkin_ws/src/speech_recognition_pkg/output.wav'
r = sr.Recognizer()
mic = sr.Microphone() #マイクの設定
output_text_file = "/home/limlab/catkin_ws/src/speech_recognition_pkg/output.txt" #出力するtxtファイルの設定
mp3_file = "/home/limlab/catkin_ws/src/speech_recognition_pkg/output.mp3"
Language_code = 'ja' #日本語
#p = pyaudio.PyAudio() #PyAudio型のインスタンス（PyAudioの関数や変数の設定などを凝縮して簡略表記したもの）作成
CHUNK = 1024 #チャンク（音源から1回読み込むときのデータサイズ。1024(=2の10乗) とする場合が多い）



def google_tts(Text): #Google TTSによる音声合成
  tts = gTTS(text=Text, lang=Language_code)
  tts.save(mp3_file)  # mp3で音声を保存する仕様
  play(mp3_file) #mp3ファイルを再生



def play(mp3_file): #mp3ファイルを再生
  p = pyaudio.PyAudio() #PyAudio型のインスタンス（PyAudioの関数や変数の設定などを凝縮して簡略表記したもの）作成
  audio_data = AudioSegment.from_mp3(mp3_file)
  FORMAT = p.get_format_from_width(audio_data.sample_width) #データフォーマット
  CHANNELS = audio_data.channels #チャンネル数
  #CHANNELS = 1 #チャンネル数
  RATE = audio_data.frame_rate #サンプリング周波数（サンプルレート、フレームレート）

  print('\nmp3_file: {}\nFORMAT: {}\nCHANNELS: {}\nRATE: {}\n').format(mp3_file, FORMAT, CHANNELS, RATE) #mp3ファイルの情報を表示

  #stream開始
  stream = p.open(format = FORMAT,
                channels = CHANNELS,
                rate = RATE,
                output = True)

  for chunk in make_chunks(audio_data, 500): #音声を再生
      stream.write(chunk._data)

  stream.stop_stream() # streamを停止
  stream.close() # streamを開放
  p.terminate() #PyAudio型には__enter__や__exit__が実装されていないため、破棄する際は明示的にterminate()メソッドを叩く必要がある



def callback(msg):
    #msg = speech_recognition_message()
    # 受信したデータを出力する
    recognized_text = msg.Text
    rospy.loginfo("受信：message = %s, count = %d" % (recognized_text, msg.count));
    google_tts(unicode(recognized_text,'utf-8')) #Google TTSによる音声合成



def subscriber():
    #初期化し、ノードの名前を設定
    rospy.init_node('play_voice', anonymous=True)
    #'recognition_txt_topic'というトピックからrecognition_text_message型のメッセージを受信する度にcallback関数を実行
    rospy.Subscriber('recognition_txt_topic', speech_recognition_message, callback)

    rospy.spin() #繰り返し(whileと同じ意味)


if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException: pass
