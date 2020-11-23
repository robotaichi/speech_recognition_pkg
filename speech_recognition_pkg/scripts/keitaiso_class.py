#!/usr/bin/env python
# -*- coding: utf-8 -*-
#上記2行は必須構文のため、コメント文だと思って削除しないこと
#Python2.7用プログラム

import rospy
from speech_recognition_pkg.msg import speech_recognition_message
import MeCab #形態素解析ライブラリ



class Subscribers(): #サブスクライバーのクラス
    def __init__(self): #コンストラクタと呼ばれる初期化のための関数（メソッド）
        #self.count = 0 
        #speech_recognition_message型のメッセージを"recognition_txt_topic"というトピックから受信するサブスクライバーの作成
        self.subscriber = rospy.Subscriber('recognition_txt_topic', speech_recognition_message, self.callback)
        self.rate = rospy.Rate(2) #1秒間に2回データを送信する



    def keitaiso(self, text): #形態素解析
        #text = unicode(text, 'utf-8')
        word = []
        hinshi = []
        mecabTagger = MeCab.Tagger("-Ochasen") #パーサーの設定。ChaSenという形態素解析器と互換の出力をする設定
        mecabTagger.parse('') #UnicodeDecodeErrorの回避
        node = mecabTagger.parseToNode(text) #textからsurface(単語)、feature(品詞情報)を解析し、その結果をnodeに代入。それぞれnode.surface、node.featureでアクセス可能
    
        while node: #ノードが存在する場合の繰り返し処理
            word.append(node.surface)#単語を抜き出す
            hinshi.append(node.feature.split(",")[0]) #データ構造：(品詞,品詞細分類1,品詞細分類2,品詞細分類3,活用形,活用型,原形,読み,発音)から品詞を抜き出す
            node = node.next #次のノードへ
        return word, hinshi



    def callback(self, message): #サブスクライバーがメッセージを受信した際に実行されるcallback関数。messageにはパブリッシャーによって配信されたメッセージ（データ）が入る
        keitaiso_info = self.keitaiso(message.Text)
        word = keitaiso_info[0]

        for i in range(len(word)): #wordの数だけ繰り返す
            rospy.loginfo("形態素解析：{}".format(word[i]));



def main(): #メイン関数
    #初期化し、ノードの名前を設定
    rospy.init_node('keitaiso_class', anonymous=True)
    #クラスのインスタンス作成（クラス内の関数や変数を使えるようにする）
    sub = Subscribers()
    rospy.spin() #callback関数を繰り返し呼び出す（終了防止）



if __name__ == '__main__':
    try:
        main() #メイン関数の実行
    except rospy.ROSInterruptException: pass