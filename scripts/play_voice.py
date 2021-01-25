#!/usr/bin/env python
# -*- coding: utf-8 -*-
#上記2行は必須構文のため、コメント文だと思って削除しないこと
#Python2.7用プログラム

import rospy
import pyaudio
from pydub import AudioSegment
from pydub.utils import make_chunks
from gtts import gTTS
from speech_recognition_pkg.msg import speech_recognition_message #メッセージファイルの読み込み（from パッケージ名.msg import 拡張子なしメッセージファイル名）
from hide_pyaudio_debug_message import noalsaerr #PyAudioのデバッグメッセージの非表示



#変数設定
mp3_file = "/home/limlab/catkin_ws/src/speech_recognition_pkg/output.mp3"
Language_code = 'ja' #日本語




class Gtts():
    def google_tts(self, Text): #Google TTSによる音声合成
        tts = gTTS(text=Text, lang=Language_code)
        tts.save(mp3_file)  # mp3で音声を保存する仕様
        self.get_mp3_file_info() #mp3ファイルの情報を取得



    def get_mp3_file_info(self): #mp3ファイルの情報を取得
        with noalsaerr():
            self.p = pyaudio.PyAudio() #PyAudio型のインスタンス（PyAudioの関数や変数の設定などを凝縮して簡略表記したもの）作成
            self.audio_data = AudioSegment.from_mp3(mp3_file)
            self.FORMAT = self.p.get_format_from_width(self.audio_data.sample_width) #データフォーマット
            self.CHANNELS = self.audio_data.channels #チャンネル数
            #CHANNELS = 1 #チャンネル数
            self.RATE = self.audio_data.frame_rate #サンプリング周波数（サンプルレート、フレームレート）
            #print('\nmp3_file: {}\nFORMAT: {}\nCHANNELS: {}\nRATE: {}\n').format(mp3_file, self.FORMAT, self.CHANNELS, self.RATE) #mp3ファイルの情報を表示
            self.play() #mp3ファイルを再生



    def play(self): #mp3ファイルを再生
        #stream開始
        stream = self.p.open(format = self.FORMAT,
                    channels = self.CHANNELS,
                    rate = self.RATE,
                    output = True)
    
        for chunk in make_chunks(self.audio_data, 500):
            stream.write(chunk._data) #音声を再生
    
        stream.stop_stream() # streamを停止
        stream.close() # streamを開放
        self.p.terminate() #PyAudio型には__enter__や__exit__が実装されていないため、破棄する際は明示的にterminate()メソッドを叩く必要がある



class Subscribers(): #サブスクライバーのクラス
    def __init__(self): #コンストラクタと呼ばれる初期化のための関数（メソッド）
        self.count = 0 
        #speech_recognition_message型のメッセージを"recognition_txt_topic"というトピックから受信するサブスクライバーの作成
        self.subscriber = rospy.Subscriber('recognition_txt_topic', speech_recognition_message, self.callback)
        self.rate = rospy.Rate(2) #1秒間に2回データを受信する
        self.gt = Gtts()



    def callback(self, message): #サブスクライバーがメッセージを受信した際に実行されるcallback関数。messageにはパブリッシャーによって配信されたメッセージ（データ）が入る
        # 受信したデータを出力する
        #rospy.loginfo("受信：message = %s, count = %d" % (message.Text, message.count));
        
        self.gt.google_tts(unicode(message.Text,'utf-8')) #Google TTSによる音声合成



def main(): #メイン関数
    #初期化し、ノードの名前を設定
    rospy.init_node('play_voice', anonymous=True)
    #クラスのインスタンス作成（クラス内の関数や変数を使えるようにする）
    sub = Subscribers()
    sub.count += 1
    rospy.spin() #callback関数を繰り返し呼び出す（終了防止）



if __name__ == '__main__':
    try:
        main() #メイン関数の実行
    except rospy.ROSInterruptException: pass