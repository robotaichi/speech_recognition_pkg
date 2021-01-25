#!/usr/bin/env python
# -*- coding: utf-8 -*-
#上記2行は必須構文のため、コメント文だと思って削除しないこと
#Python2.7用プログラム

import rospy
from speech_recognition_pkg.msg import speech_recognition_message #メッセージファイルの読み込み（from パッケージ名.msg import 拡張子なしメッセージファイル名）
import speech_recognition as sr
import sys

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
        self.rate = rospy.Rate(0.4) #1秒間に0.4回データを送信する
        self.words = ["apple","banana","melon","contour","conceal"]
        self.meanings = ["りんご","バナナ", "メロン","輪郭","覆う"]



    def explain(self): #説明
            Text = unicode("{}は「{}」という意味です。".format(self.words[self.count],self.meanings[self.count]), 'utf-8')
            return Text


    def make_msg(self): #送信するメッセージの作成
            self.message.Text = u"{}".format(self.explain()).encode("utf-8") #voice_record関数により音声認識した結果を日本語も扱えるutf-8型にエンコード
            self.message.count = self.count



    def send_msg(self): #メッセージを送信
        self.make_msg() #送信するメッセージの作成
        self.publisher.publish(self.message) #作成したメッセージの送信
        rospy.loginfo("送信：message = %s, count = %d" %(self.message.Text, self.message.count))



def main(): #メイン関数
    #初期化し、ノードの名前を設定
    rospy.init_node('explain_node', anonymous=True)
    #クラスのインスタンス作成（クラス内の関数や変数を使えるようにする）
    pub = Publishsers()
    
    while not rospy.is_shutdown(): #Ctrl + Cが押されるまで繰り返す
        pub.rate.sleep()
        pub.send_msg() #メッセージを送信
        pub.rate.sleep()
        pub.count += 1
        if pub.count >= len(pub.words):
            sys.exit()


if __name__ == '__main__':
    try:
        main() #メイン関数の実行
    except rospy.ROSInterruptException: pass