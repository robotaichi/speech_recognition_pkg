#!/usr/bin/env python
# -*- coding: utf-8 -*-
#上記2行は必須構文のため、コメント文だと思って削除しないこと
#Python2.7用プログラム

import rospy
from speech_recognition_pkg.msg import speech_recognition_message #メッセージファイルの読み込み（from パッケージ名.msg import 拡張子なしメッセージファイル名）
import speech_recognition as sr
import sys
import MeCab

#変数設定
Language = 'ja-JP' #音声認識の対象言語を設定
r = sr.Recognizer()
mic = sr.Microphone() #マイクの設定

words = ["conceal", "contract"] #単語リスト
meanings = ["覆う", "引き合う"] #意味リスト
element_group_number = 2 #配列を()ごとにまとめる際の()に含まれる要素数



class Mecab():
    def __init__(self):
        self.word_with_blank = []
        self.word_without_blank = []
        self.gogen_with_blank = []
        self.gogen_without_blank = []
        self.list_with_blank1 = []
        self.list_with_blank2 = []
        self.list_without_blank1 = []
        self.list_without_blank2 = []
        self.output = []
        self.first = True



    def with_blank(self, node):
        while node:
            self.list_with_blank1.append(node.surface)
            word_class = node.feature.split(',') #word_classの構造：品詞, 品詞細分類1, 品詞細分類2, 品詞細分類3, 活用形, 活用型, 原形, 読み, 発音
            self.list_with_blank2.append(word_class[6].replace("*", "")) #「*」を消去
            #hinshi = self.node.feature.split(",")[0]
            node = node.next
        return self.list_with_blank1, self.list_with_blank2



    def without_blank(self, list_with_blank1, list_with_blank2):
        for element in list_with_blank1:
            if element != '': #形態素がある（要素が存在する）場合
                self.list_without_blank1.append(element)
        for element in list_with_blank2:
            if element != '': #語源がある（要素が存在する）場合
                self.list_without_blank2.append(element)
        return self.list_without_blank1, self.list_without_blank2



    def combine(self, list_without_blank1, list_without_blank2):
        for i in range(len(list_without_blank1)): #単語と語源の配列を連結
            self.output.append(list_without_blank1[i])
            self.output.append(list_without_blank2[i])
        self.output = zip(*[iter(self.output)]*element_group_number) #element_group_numberごとに配列の中身を()でまとめる



    def execute_node(self, node):
        self.word_with_blank, self.gogen_with_blank = self.with_blank(node)
        self.word_without_blank, self.gogen_without_blank = self.without_blank(self.word_with_blank, self.gogen_with_blank)
        if len(self.gogen_without_blank) == 0: #語源がない（配列の要素が全て空の）場合
            self.gogen_without_blank.append(None)
        self.combine(self.word_without_blank, self.gogen_without_blank)



    def print_output(self, number):
        text1 = "\n{}は\n".format(words[number])
        text2 = []
        for i in range(len(self.output)):
            if self.output[i][1] != None:
                text2.append("{}が「{}」\n".format(self.output[i][0], self.output[i][1]))
        for i in range(len(self.output)):
            if (self.output[i][1] != None) and (self.first == True):
                text3 = "から「{}」という意味になります\n".format(meanings[number])
                self.first = False
            elif self.output[i][1] == None:
                text3 = "「{}」という意味です\n".format(meanings[number])
        text2 = "".join(text2) #配列の文字列要素を連結
        Text = text1 + text2 + text3
        return Text



    def mecab_main(self, count):
        mecabTagger = MeCab.Tagger("-Ochasen \ -d /usr/local/lib/mecab/dic/ipadic \ -u /usr/local/lib/mecab/dic/userdic/gogen.dic") # -d(--dicdir)：使用するシステム辞書を指定 -u(--userdic):ユーザ辞書を指定
        mecabTagger.parse("")  # Unicode Decode Errorの回避
        node = mecabTagger.parseToNode(words[count])
        self.execute_node(node)
        Text = self.print_output(count)
        return Text



class Publishsers(): #パブリッシャーのクラス
    def __init__(self): #コンストラクタと呼ばれる初期化のための関数（メソッド）
        self.count = 0
        #messageの型を作成
        self.message = speech_recognition_message() 
        #speech_recognition_message型のメッセージを"recognition_txt_topic"というトピックに送信するパブリッシャーの作成
        self.publisher = rospy.Publisher('recognition_txt_topic', speech_recognition_message, queue_size=10)
        self.rate = rospy.Rate(0.4) #1秒間に0.4回データを送信する



    def explain(self): #説明
            mc = Mecab()
            Text = mc.mecab_main(self.count)
            print(Text)
            return Text


    def make_msg(self): #送信するメッセージの作成
            self.message.Text = "{}".format(self.explain())
            self.message.count = self.count



    def send_msg(self): #メッセージを送信
        self.make_msg() #送信するメッセージの作成
        self.publisher.publish(self.message) #作成したメッセージの送信
        #rospy.loginfo("送信：message = %s, count = %d" %(self.message.Text, self.message.count))



def main(): #メイン関数
    #初期化し、ノードの名前を設定
    rospy.init_node('explain_gogen_node', anonymous=True)
    #クラスのインスタンス作成（クラス内の関数や変数を使えるようにする）
    pub = Publishsers()
    
    while not rospy.is_shutdown(): #Ctrl + Cが押されるまで繰り返す
        pub.rate.sleep()
        pub.send_msg() #メッセージを送信
        pub.rate.sleep()
        pub.count += 1
        if pub.count >= len(words):
            sys.exit()



if __name__ == '__main__':
    try:
        main() #メイン関数の実行
    except rospy.ROSInterruptException: pass