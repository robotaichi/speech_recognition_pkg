#!/usr/bin/env python
# -*- coding: utf-8 -*-
#上記2行は必須構文のため、コメント文だと思って削除しないこと
#Python2.7用プログラム

import rospy
import MeCab
import json
from pydub import AudioSegment
import subprocess #Python上からLinuxのシェルコマンドを実行するためのモジュール
import os
import wave 
import pyaudio
from hide_pyaudio_debug_message import noalsaerr #PyAudioのデバッグメッセージの非表示
from speech_recognition_pkg.msg import speech_recognition_message #メッセージファイルの読み込み（from パッケージ名.msg import 拡張子なしメッセージファイル名）
import alkana #アルファベットをカタカナに変換するライブラリ

file_path = os.path.join(os.path.dirname(__file__), "open_jtalk")



class CommonModule:
    def load_json(self, file):
        with open(file, 'r') as f:
            json_object = json.load(f)
        return json_object



class NLP:
    def __init__(self):
        self.cm = CommonModule()



    def morphological_analysis(self, text, keyword='-Ochasen'):
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



    def evaluate_pn_ja_wordlist(self, wordlist, speaklist):
        word_pn_dict = self.cm.load_json(os.path.join(os.path.dirname(__file__), "pn_ja.json"))

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
            alkana_word = alkana.get_kana(text) #アルファベットをカタカナに変換
            if alkana_word != None: #alkanaの辞書にそのアルファベットが存在する場合
                text = alkana_word #speak_listの要素をカタカナに置き換える
            speak_list.append(text)
        text = "".join(speak_list) #配列の文字列要素を連結
        # for i in range(len(speak_list)):
        #     print(speak_list[i])
        return pn_value, text



    def set_key_and_value(self, word_pn_dict): #キーリストと値リストの取得
        key_list = []
        value_list = []
        json_dict = dict()
        for key, value in word_pn_dict.items(): #json_objectからkeyとvalueを取得し、その数だけ繰り返す
            if isinstance(key, unicode): #keyがユニコード型の場合
                key = str(u"{}".format(key).encode("utf-8")) #文字列に変換
            if isinstance(value, unicode): #valueがユニコード型の場合
                value = str(u"{}".format(value).encode("utf-8")) #文字列に変換
            key_list.append(key) #keyをkey_listに追加
            value_list.append(value) #valueをvalue_listに追加
        for i in range(len(key_list)): #key_listの要素数だけ繰り返す
            json_dict[key_list[i]] = value_list[i] #params辞書の設定
        return json_dict



    def evaluate_pn_ja_word(self, word, word_pn_dict):
        if type(word) is dict:
            word = word['base']
        elif type(word) is str:
            pass
        else:
            raise TypeError

        json_dict = self.set_key_and_value(word_pn_dict)

        if word in json_dict:
            word = unicode(word,"utf-8") #utf-8に変換
            pn_value = float(word_pn_dict[word]['value'])
            return pn_value
        return 0



    def analysis_emotion(self, text):
            split_words, speak_words = self.morphological_analysis(text, "-Ochasen")
            pn_value, text = self.evaluate_pn_ja_wordlist(split_words, speak_words)
            if pn_value < -0.8:
                emotion = 'angry'
            elif (-0.8 <= pn_value) and (pn_value < -0.5):
                emotion = 'sad'
            elif (-0.8 <= pn_value) and (pn_value <= -0.5):
                emotion = 'normal'
            else: #pn_value > 0.5
                emotion = 'happy'
            return emotion, text



class Jtalk():
    def __init__(self):
        self.conf = {
            "voice_configs": {
                "htsvoice_resource": "/usr/share/hts-voice/",
                "jtalk_dict": "/var/lib/mecab/dic/open-jtalk/naist-jdic"
            }
        }



    def jtalk(self, text, filepath, voicetype='mei', emotion='normal'):
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
        c = subprocess.Popen(cmd, stdin=subprocess.PIPE) #標準入力を指定してコマンド(cmd)を実行。プロセスを生成するだけで、終了を待たない
        c.stdin.write(text)
        c.stdin.close()
        c.wait() #コマンド終了を待機
        audio_file = filepath+'.wav'
        self.get_audio_file_info(audio_file) #音声ファイルの情報を取得



    def get_audio_file_info(self, audio_file): #音声ファイルの情報を取得
        with noalsaerr():
            self.p = pyaudio.PyAudio() #PyAudio型のインスタンス（PyAudioの関数や変数の設定などを凝縮して簡略表記したもの）作成
            self.audio_data = AudioSegment.from_wav(audio_file)
            self.FORMAT = self.p.get_format_from_width(self.audio_data.sample_width) #データフォーマット
            self.CHANNELS = self.audio_data.channels #チャンネル数
            #CHANNELS = 1 #チャンネル数
            self.RATE = self.audio_data.frame_rate #サンプリング周波数（サンプルレート、フレームレート）
            # print('\naudio_file: {}\nFORMAT: {}\nCHANNELS: {}\nRATE: {}\n').format(audio_file, self.FORMAT, self.CHANNELS, self.RATE) #mp3ファイルの情報を表示
            self.play(audio_file) #wavファイルを再生



    def play(self, audio_file): #wavファイルを再生
        wf = wave.open(audio_file, "r")
        chunk = 1024
        #stream開始
        stream = self.p.open(format = self.FORMAT,
                    channels = self.CHANNELS,
                    rate = self.RATE,
                    output = True)
    
        data = wf.readframes(chunk)
        while data != '': #音声データが存在する間
            stream.write(data) #音声を再生
            data = wf.readframes(chunk)
    
        stream.stop_stream() # streamを停止
        stream.close() # streamを開放
        self.p.terminate() #PyAudio型には__enter__や__exit__が実装されていないため、破棄する際は明示的にterminate()メソッドを叩く必要がある



class Subscribers(): #サブスクライバーのクラス
    def __init__(self): #コンストラクタと呼ばれる初期化のための関数（メソッド）
        self.count = 0 
        #speech_recognition_message型のメッセージを"recognition_txt_topic"というトピックから受信するサブスクライバーの作成
        self.subscriber = rospy.Subscriber('recognition_txt_topic', speech_recognition_message, self.callback)
        self.rate = rospy.Rate(2) #1秒間に2回データを受信する



    def callback(self, message): #サブスクライバーがメッセージを受信した際に実行されるcallback関数。messageにはパブリッシャーによって配信されたメッセージ（データ）が入る
        # 受信したデータを出力する
        #rospy.loginfo("受信：message = %s, count = %d" % (message.Text, message.count));
        
        nlp = NLP()
        emotion, text = nlp.analysis_emotion(message.Text)
        jt = Jtalk()
        jt.jtalk(text, file_path, emotion=emotion)



def main(): #メイン関数
    #初期化し、ノードの名前を設定
    rospy.init_node('openjtalk', anonymous=True)
    #クラスのインスタンス作成（クラス内の関数や変数を使えるようにする）
    sub = Subscribers()
    sub.count += 1
    rospy.spin() #callback関数を繰り返し呼び出す（終了防止）



if __name__ == '__main__':
    try:
        main() #メイン関数の実行
    except rospy.ROSInterruptException: pass