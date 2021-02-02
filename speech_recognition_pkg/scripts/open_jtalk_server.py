#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 上記2行は必須構文のため、コメント文だと思って削除しないこと
# Python2.7用プログラム

import rospy #ROSをPythonで扱うのに必要
import MeCab #Mecabの形態素解析に使用
import json #Jsonファイルを扱うのに必要
from pydub import AudioSegment
import subprocess  # Python上からLinuxのシェルコマンドを実行するためのモジュール
import os #ファイルパスを扱うのに必要
import wave #wavファイルを扱うのに必要
import pyaudio #wavファイルの再生に必要
from hide_pyaudio_debug_message import noalsaerr  # PyAudioのデバッグメッセージの非表示
# メッセージファイルの読み込み（from パッケージ名.msg import 拡張子なしメッセージファイル名）
from speech_recognition_pkg.msg import speech_recognition_message
# サービスファイルの読み込み（from パッケージ名.srv import 拡張子なしサービスファイル名）
from speech_recognition_pkg.srv import play_end_check_service, openjtalk_service
import alkana  # アルファベットをカタカナに変換するライブラリ

file_path = os.path.join(os.path.dirname(__file__), "open_jtalk")



class CommonModule: #OpenJTalkの共通モジュール
    def load_json(self, file): #Jsonファイルの読み込み
        with open(file, 'r') as f:
            json_object = json.load(f)
        return json_object



class NLP: #自然言語処理(Natual Language Processing)クラス
    def __init__(self):
        self.cm = CommonModule() #OpenJTalkの共通モジュールのインスタンス生成



    def morphological_analysis(self, text, keyword='-Ochasen'): #形態素解析
        words = []
        speak_list = []
        tagger = MeCab.Tagger(keyword)
        result = tagger.parse(text)
        result = result.split('\n')
        result = result[:-2]

        for word in result:
            temp = word.split('\t')
            # print(word)
            word_info = {
                'surface': temp[0],
                'kana': temp[1],
                'base': temp[2],
                'pos': temp[3],
                'conjugation': temp[4],
                'form': temp[5]
            }
            words.append(word_info)
            speak_list.append(temp[2])
        return words, speak_list



    def evaluate_pn_ja_wordlist(self, wordlist, speaklist): #単語リストの感情値の評価
        word_pn_dict = self.cm.load_json(os.path.join(
            os.path.dirname(__file__), "pn_ja.json")) #Jsonファイルの読み込み

        pn_value = 0
        speak_list = []
        for word in wordlist:
            pn_value = self.evaluate_pn_ja_word(word, word_pn_dict)
            # if pn_value >= 0.5:
            #     emotion = 'happy'
            # elif pn_value <= -0.8:
            #     emotion = 'angry'
            # elif (-0.8 < pn_value) and (pn_value < -0.5):
            #     emotion = 'sad'
            # else:
            #     emotion = 'normal'
            # print("「{}」の感情：{}({})".format(word['base'], emotion, pn_value))
        for text in speaklist:
            alkana_word = alkana.get_kana(text)  # アルファベットをカタカナに変換
            if alkana_word != None:  # alkanaの辞書にそのアルファベットが存在する場合
                text = alkana_word  # speak_listの要素をカタカナに置き換える
            speak_list.append(text)
        text = "".join(speak_list)  # 配列の文字列要素を連結
        # for i in range(len(speak_list)):
        #     print(speak_list[i])
        return pn_value, text



    def set_key_and_value(self, word_pn_dict):  # キーリストと値リストの取得
        key_list = []
        value_list = []
        json_dict = dict()
        for key, value in word_pn_dict.items():  # json_objectからkeyとvalueを取得し、その数だけ繰り返す
            if isinstance(key, unicode):  # keyがユニコード型の場合
                key = str(u"{}".format(key).encode("utf-8"))  # 文字列に変換
            if isinstance(value, unicode):  # valueがユニコード型の場合
                value = str(u"{}".format(value).encode("utf-8"))  # 文字列に変換
            key_list.append(key)  # keyをkey_listに追加
            value_list.append(value)  # valueをvalue_listに追加
        for i in range(len(key_list)):  # key_listの要素数だけ繰り返す
            json_dict[key_list[i]] = value_list[i]  # params辞書の設定
        return json_dict



    def evaluate_pn_ja_word(self, word, word_pn_dict): #単語の感情値の評価
        if type(word) is dict:
            word = word['base']
        elif type(word) is str:
            pass
        else:
            raise TypeError

        json_dict = self.set_key_and_value(word_pn_dict) # キーリストと値リストの取得

        if word in json_dict:
            word = unicode(word, "utf-8")  # utf-8に変換
            pn_value = float(word_pn_dict[word]['value'])
            return pn_value
        return 0



    def analysis_emotion(self, text): #表情解析
        split_words, speak_words = self.morphological_analysis(
            text, "-Ochasen") #形態素解析
        pn_value, text = self.evaluate_pn_ja_wordlist(split_words, speak_words) #単語リストの感情値の評価
        if pn_value < -0.8:
            emotion = 'angry'
        elif (-0.8 <= pn_value) and (pn_value < -0.5):
            emotion = 'sad'
        elif (-0.8 <= pn_value) and (pn_value <= -0.5):
            emotion = 'normal'
        else:  # pn_value > 0.5
            emotion = 'happy'
        return emotion, text



class Jtalk(): #OpenJTalkのクラス
    def __init__(self):
        self.conf = {
            "voice_configs": {
                "htsvoice_resource": "/usr/share/hts-voice/",
                "jtalk_dict": "/var/lib/mecab/dic/open-jtalk/naist-jdic"
            }
        }
        self.play_end = False



    def jtalk(self, text, filepath, voicetype='mei', emotion='normal'): #音声合成
        htsvoice = {
            'mei': {
                'normal': ['-m', os.path.join(self.conf['voice_configs']['htsvoice_resource'], 'mei/mei_normal.htsvoice')],
                'angry': ['-m', os.path.join(self.conf['voice_configs']['htsvoice_resource'], 'mei/mei_angry.htsvoice')],
                'bashful': ['-m', os.path.join(self.conf['voice_configs']['htsvoice_resource'], 'mei/mei_bashful.htsvoice')],
                'happy': ['-m', os.path.join(self.conf['voice_configs']['htsvoice_resource'], 'mei/mei_happy.htsvoice')],
                'sad': ['-m', os.path.join(self.conf['voice_configs']['htsvoice_resource'], 'mei/mei_sad.htsvoice')]
            },
            'm100': {
                'normal': ['-m', os.path.join(self.conf['voice_configs']['htsvoice_resource'], 'm100/nitech_jp_atr503_m001.htsvoice')]
            },
            'tohoku-f01': {
                'normal': ['-m', os.path.join(self.conf['voice_configs']['htsvoice_resource'], 'htsvoice-tohoku-f01-master/tohoku-f01-neutral.htsvoice')],
                'angry': ['-m', os.path.join(self.conf['voice_configs']['htsvoice_resource'], 'htsvoice-tohoku-f01-master/tohoku-f01-angry.htsvoice')],
                'happy': ['-m', os.path.join(self.conf['voice_configs']['htsvoice_resource'], 'htsvoice-tohoku-f01-master/tohoku-f01-happy.htsvoice')],
                'sad': ['-m', os.path.join(self.conf['voice_configs']['htsvoice_resource'], 'htsvoice-tohoku-f01-master/tohoku-f01-sad.htsvoice')]
            }
        }

        open_jtalk = ['open_jtalk']
        mech = ['-x', self.conf['voice_configs']['jtalk_dict']]
        speed = ['-r', '1.0']
        outwav = ['-ow', filepath+'.wav']
        cmd = open_jtalk + mech + htsvoice[voicetype][emotion] + speed + outwav
        # 標準入力を指定してコマンド(cmd)を実行。プロセスを生成するだけで、終了を待たない
        c = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        c.stdin.write(text)
        c.stdin.close()
        c.wait()  # コマンド終了を待機
        audio_file = filepath+'.wav'
        self.get_audio_file_info(audio_file)  # 音声ファイルの情報を取得
        self.play(audio_file) #wavファイルの再生



    def get_audio_file_info(self, audio_file):  # 音声ファイルの情報を取得
        with noalsaerr():
            self.p = pyaudio.PyAudio()  # PyAudio型のインスタンス（PyAudioの関数や変数の設定などを凝縮して簡略表記したもの）作成
            self.audio_data = AudioSegment.from_wav(audio_file)
            self.FORMAT = self.p.get_format_from_width(
                self.audio_data.sample_width)  # データフォーマット
            self.CHANNELS = self.audio_data.channels  # チャンネル数
            # CHANNELS = 1 #チャンネル数
            self.RATE = self.audio_data.frame_rate  # サンプリング周波数（サンプルレート、フレームレート）
            # print('\naudio_file: {}\nFORMAT: {}\nCHANNELS: {}\nRATE: {}\n').format(audio_file, self.FORMAT, self.CHANNELS, self.RATE) #mp3ファイルの情報を表示



    def play(self, audio_file):  # wavファイルを再生
        wf = wave.open(audio_file, "r")
        chunk = 1024
        # stream開始
        stream = self.p.open(format=self.FORMAT, channels=self.CHANNELS, rate=self.RATE, output=True)

        data = wf.readframes(chunk)
        while data != '':  # 音声データが存在する間
            stream.write(data)  # 音声を再生
            data = wf.readframes(chunk)

        stream.stop_stream()  # streamを停止
        stream.close()  # streamを開放
        # PyAudio型には__enter__や__exit__が実装されていないため、破棄する際は明示的にterminate()メソッドを叩く必要がある
        self.p.terminate()



class OpenJTalk_Server():  # サーバーのクラス
    def __init__(self):
        # service_messageの型を作成
        self.service_message = openjtalk_service()
        self.service_message.openjtalk = False
        self.rate = rospy.Rate(0.2)  # 1秒間に0.2回データを受信する
        # self.jt = Jtalk()



    def make_response(self): #リスポンスの作成
            self.service_message.openjtalk_response = True



    def success_log(self, req):  # 成功メッセージの表示（callback関数）
        #rospy.loginfo("\nOpenJTalkサービスのリクエストがありました：\nmessage = {}\n".format(req.openjtalk_request))  # クライアント側で使用するサービスの定義
        pub = Publishsers() #パブリッシャのクラスのインスタンス生成（if文の中に入れると送信できないので注意）
        nlp = NLP()  # クラスのインスタンス生成
        emotion, text = nlp.analysis_emotion(req.openjtalk_request)  # 形態素から感情分析
        jt = Jtalk()  # クラスのインスタンス生成
        jt.jtalk(text, file_path, emotion=emotion)  # 音声合成かつ最後まで再生
        if "類義語" in req.openjtalk_request: # 受信したreq.openjtalk_requestに「類義語」が含まれている場合
            srv = Play_End_Check_Server() #クラスのインスタンス生成
            srv.service_response() #サービスの応答
        if "意味になります" in req.openjtalk_request: # 受信したreq.openjtalk_requestに「意味になります」か「意味です」が含まれている場合
            pub.send_msg() #メッセージの送信
        if "意味です" in req.openjtalk_request: # 受信したreq.openjtalk_requestに「意味になります」か「意味です」が含まれている場合
            pub.send_msg() #メッセージの送信
        self.make_response()
        return self.service_message.openjtalk_response



    def service_response(self):  # サービスの応答
        # サービスのリクエストがあった場合にsuccess_log関数（callback関数）を呼び出し、実行。呼び出し先の関数内で返り値をreturnする必要がある
        srv = rospy.Service('openjtalk_service',
                            openjtalk_service, self.success_log)




class Play_End_Check_Server():  # サーバーのクラス
    def __init__(self):
        # service_messageの型を作成
        self.service_message = play_end_check_service()
        self.service_message.play_end_check_response = False
        self.rate = rospy.Rate(1)  # 1秒間に1回データを受信する
        # self.jt = Jtalk()



    def make_response(self): #リクエストの作成
            self.service_message.play_end_check_response = True



    def success_log(self, req):  # 成功メッセージの表示（callback関数）
        rospy.loginfo("\n再生状況確認サービスのリクエストがありました：\nmessage = {}\n".format(req.play_end_check_request))  # クライアント側で使用するサービスの定義
        self.make_response()
        # srvファイルで定義した返り値をsrvに渡す。rospy.Serviceによって呼び出された関数（callback関数）内でreturnすること
        return self.service_message.play_end_check_response



    def service_response(self):  # サービスの応答
        # サービスのリクエストがあった場合にsuccess_log関数（callback関数）を呼び出し、実行。呼び出し先の関数内で返り値をreturnする必要がある
        srv = rospy.Service('play_end_check_service',
                            play_end_check_service, self.success_log)



class Publishsers():  # パブリッシャーのクラス
    def __init__(self):  # コンストラクタと呼ばれる初期化のための関数（メソッド）
        self.count = 0
        # messageの型を作成
        self.message = speech_recognition_message()
        # speech_recognition_message型のメッセージを"realsense_tf_topic"というトピックに送信するパブリッシャーの作成
        self.realsense_tf_publisher = rospy.Publisher(
            'realsense_tf_topic', speech_recognition_message, queue_size=10)
        self.rate = rospy.Rate(10)  # 1秒間に10回データを送信する



    def make_realsense_tf(self):  # 送信するメッセージの作成
        self.message.realsense_tf = True



    def send_msg(self):  # メッセージを送信
        self.make_realsense_tf()
        self.realsense_tf_publisher.publish(self.message)  # 作成したメッセージの送信
        # rospy.loginfo("realsense_tfを送信:{}".format(self.message.realsense_tf))



def main():  # メイン関数
    # 初期化し、ノードの名前を設定
    rospy.init_node('open_jtalk_server', anonymous=True)
    ojs = OpenJTalk_Server() #クラスのインスタンス作成（クラス内の関数や変数を使えるようにする）
    ojs.service_response() #サービスの応答
    rospy.spin()  # callback関数を繰り返し呼び出す（終了防止）



if __name__ == '__main__':
    try:
        main()  # メイン関数の実行
    except rospy.ROSInterruptException:
        pass
