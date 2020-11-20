#!/usr/bin/env python
# -*- coding: utf-8 -*-
#上記2行は必須構文のため、コメント文だと思って削除しないこと
#Python2.7用プログラム

import rospy
from speech_recognition_pkg.msg import speech_recognition_message
import speech_recognition as sr

#変数設定
Language = 'ja-JP' #音声認識の対象言語を設定
r = sr.Recognizer()
mic = sr.Microphone() #マイクの設定



class Publishsers(): #パブリッシャーのクラス
    def __init__(self): #コンストラクタと呼ばれる初期化のための関数（メソッド）
        self.count = 0
        #messageの型を作成
        self.message = speech_recognition_message() 
        #speech_recognition_message型のメッセージを"recognition_txt_topic"というトピックに送信するパブリッシャーの作成
        self.publisher = rospy.Publisher('recognition_txt_topic', speech_recognition_message, queue_size=10)
        self.rate = rospy.Rate(2) #1秒間に2回データを送信する



    def voice_record(self): #音声録音
        while True:
            print("録音開始")
        
            with mic as source: #マイクをソースとして開き、終了後自動的に閉じる
                r.adjust_for_ambient_noise(source) #雑音対策
                audio = r.listen(source) #音声を録音
        
            print("録音終了\n")
        
            try: #音声を認識できた場合
                Text = r.recognize_google(audio, language = Language) #音声認識
                print(u'Transcript: {}'.format(Text).encode('utf-8')) #認識した音声（テキスト）を表示
        
            #音声を認識できなかった場合
            except sr.UnknownValueError:
                Text = unicode("音声を認識できませんでした。もう一度お話ください", 'utf-8')
                print("音声を認識できませんでした。もう一度お話ください\n")
            except sr.RequestError as e:
                Text = unicode("Google Speech Recognitionサービスからの結果をリクエストできませんでした", 'utf-8')
                print("Google Speech Recognitionサービスからの結果をリクエストできませんでした: {0}".format(e))

            return Text



    def make_msg(self): #送信するメッセージの作成
            self.message.Text = u"{}".format(self.voice_record()).encode("utf-8") #voice_record関数により音声認識した結果を日本語も扱えるutf-8型にエンコード
            self.message.count = self.count



    def send_msg(self): #メッセージを送信
        self.make_msg() #送信するメッセージの作成
        self.publisher.publish(self.message) #作成したメッセージの送信
        rospy.loginfo("送信：message = %s, count = %d" %(self.message.Text, self.message.count))



def main(): #メイン関数
    #初期化し、ノードの名前を設定
    rospy.init_node('speech_recognition_publisher', anonymous=True)
    #クラスのインスタンス作成（クラス内の関数や変数を使えるようにする）
    pub = Publishsers()
    
    while not rospy.is_shutdown(): #Ctrl + Cが押されるまで繰り返す
        pub.send_msg() #メッセージを送信
        pub.rate.sleep()
        pub.count += 1


if __name__ == '__main__':
    try:
        main() #メイン関数の実行
    except rospy.ROSInterruptException: pass