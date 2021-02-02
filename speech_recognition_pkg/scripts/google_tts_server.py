#!/usr/bin/env python
# -*- coding: utf-8 -*-
#上記2行は必須構文のため、コメント文だと思って削除しないこと
#Python2.7用プログラム

import rospy #ROSをPythonで扱うのに必要
import pyaudio #wavファイルを扱うのに必要
from pydub import AudioSegment #mp3ファイルを扱うのに必要
from pydub.utils import make_chunks #mp3ファイルを扱うのに必要
from gtts import gTTS #Google TTSを使用するのに必要
from speech_recognition_pkg.srv import google_tts_service # サービスファイルの読み込み（from パッケージ名.srv import 拡張子なしサービスファイル名）
from hide_pyaudio_debug_message import noalsaerr #PyAudioのデバッグメッセージの非表示

#変数設定
mp3_file = "/home/limlab/catkin_ws/src/speech_recognition_pkg/output.mp3"
Language_code = 'ja' #日本語



class Google_TTS_Server(): #サーバーのクラス
    def __init__(self):
        #service_messageの型を作成
        self.service_message = google_tts_service()
        self.rate = rospy.Rate(0.3) # 1秒間に0.3回データを受信する



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



    def make_response(self): #リスポンスの作成
        self.service_message.google_tts_response = True



    def success_log(self, req): #成功メッセージの表示（callback関数）
        self.google_tts(unicode(req.Text,'utf-8')) #Google TTSによる音声合成
        self.make_response()
        return self.service_message.google_tts_response #srvファイルで定義した返り値をsrvに渡す。rospy.Serviceによって呼び出された関数（callback関数）内でreturnすること



    def service_response(self): #サービスの応答
        srv = rospy.Service('google_tts_service', google_tts_service, self.success_log) #サービスのリクエストがあった場合にsuccess_log関数（callback関数）を呼び出し、実行。呼び出し先の関数内で返り値をreturnする必要がある



def main(): #メイン関数
    #初期化し、ノードの名前を設定
    rospy.init_node('google_tts_server', anonymous=True)
    gts = Google_TTS_Server() #クラスのインスタンス作成（クラス内の関数や変数を使えるようにする）
    gts.service_response() #サービスの応答
    rospy.spin() #callback関数を繰り返し呼び出す（終了防止）



if __name__ == '__main__':
    try:
        main() #メイン関数の実行
    except rospy.ROSInterruptException: pass