#!/usr/bin/env python
# -*- coding: utf-8 -*-
#上記2行は必須構文のため、コメント文だと思って削除しないこと
#Python2.7用プログラム

import rospy
from speech_recognition_pkg.msg import speech_recognition_message #メッセージファイルの読み込み（from パッケージ名.msg import 拡張子なしメッセージファイル名）
import speech_recognition as sr
import sys
import MeCab
import sqlite3
import requests
import random
import shutil
import bs4
import os
import ssl
ssl._create_default_https_context = ssl._create_unverified_context
import cv2

#変数設定
Language = 'ja-JP' #音声認識の対象言語を設定
r = sr.Recognizer()
mic = sr.Microphone() #マイクの設定

words = ["conceal", "contract", "contest"] #単語リスト
meanings = ["覆う", "引き合う", "コンテスト"] #意味リスト
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
        self.Text_list = []
        self.converted_Text_list = []
        self.first = True
        self.ctr = 0



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
        text1 = "{}は\n".format(words[number])
        self.Text_list.append(text1)
        text2_list = []
        search_word_list = []
        for i in range(len(self.output)):
            if self.output[i][1] != None: #語源がある（要素が存在する）場合
                wn = Wordnet()
                synonym = wn.search_similar_words(self.mecab_for_wordnet(self.output[i][1]))
                text2_list.append("{}が「{}」\n類義語が{}\n".format(self.output[i][0], self.output[i][1], synonym))
                self.Text_list.append(text2_list[i])
                search_word_list.append(self.output[i][1])
        for i in range(len(self.output)):
            if (self.output[i][1] != None) and (self.first == True): #語源がある（要素が存在する）場合
                text3 = "から「{}」という意味になります\n".format(meanings[number])
                self.first = False
            elif self.output[i][1] == None: #語源がない（配列の要素が全て空の）場合
                text3 = "「{}」という意味です\n".format(meanings[number])
        self.Text_list.append(text3)
        return self.Text_list, search_word_list



    def mecab_for_wordnet(self, text):
        tagger = MeCab.Tagger("-Ochasen \ -d /usr/local/lib/mecab/dic/ipadic \ -u /usr/local/lib/mecab/dic/userdic/gogen.dic") # -d(--dicdir)：使用するシステム辞書を指定 -u(--userdic):ユーザ辞書を指定
        tagger.parse('') # Unicode Decode Errorの回避
        word_list_with_blank = []
        word_list_without_blank = []
        node = tagger.parseToNode(text)
        while node:
            word = node.surface
            word_list_with_blank.append(word)
            node = node.next
        for element in word_list_with_blank:
            if element != '': #形態素がある（要素が存在する）場合
                word_list_without_blank.append(element)
        return word_list_without_blank[0]



    def mecab_main(self, count):
        mecabTagger = MeCab.Tagger("-Ochasen \ -d /usr/local/lib/mecab/dic/ipadic \ -u /usr/local/lib/mecab/dic/userdic/gogen.dic") # -d(--dicdir)：使用するシステム辞書を指定 -u(--userdic):ユーザ辞書を指定
        mecabTagger.parse("")  # Unicode Decode Errorの回避
        node = mecabTagger.parseToNode(words[count])
        self.execute_node(node)
        Text_list, search_word_list = self.print_output(count)
        return Text_list, search_word_list



class Wordnet():
    def __init__(self):
        self.conn = sqlite3.connect("/home/limlab/Programs/wordnet/wnjpn.db")



    def search_similar_words(self, word): # 特定の単語を入力とした時に、類義語を検索する関数
        # 問い合わせしたい単語がWordnetに存在するか確認する
        cur = self.conn.execute("select wordid from word where lemma='%s'" %word)
        word_id = 99999999  #一時的なID
        for row in cur:
            word_id = row[0]
    
        # Wordnetに存在する語であるかの判定
        if word_id==99999999:
            print("「%s」は、Wordnetに存在しない単語です。" %word)
            return
    
        # 入力された単語を含む概念を検索する
        cur = self.conn.execute("select synset from sense where wordid='%s'" % word_id)
        synsets = []
        synonyms = []
        for row in cur:
            synsets.append(row[0])
    
        # 概念に含まれる単語を検索して画面出力する
        for synset in synsets:
            cur1 = self.conn.execute("select name from synset where synset='%s'" % synset)
            for row1 in cur1:
                synonyms.append(row1[0])
        return synonyms[0]



class Google_image():
    def __init__(self):
        self.file_name = "/home/limlab/catkin_ws/src/speech_recognition_pkg/scripts/download_image.png"



    def create_url(self, data): #Google画像検索のURLを生成
        while True:
            Res = requests.get("https://www.google.com/search?hl=jp&q=" + data + "&btnG=Google+Search&tbs=0&safe=on&tbm=isch")
            Html = Res.text
            Soup = bs4.BeautifulSoup(Html,'lxml')
            links = Soup.find_all("img")
            link = random.choice(links).get("src")
            if link != "/images/branding/searchlogo/1x/googlelogo_desk_heirloom_color_150x55dp.gif": #Gif画像でない場合
                print("{}\n".format(link))
                return link
            # else:
            #     print("Gif画像のURLを取得しました。再取得します")
    
    
    
    def download_img(self, url): #Google画像検索から画像をダウンロード
        r = requests.get(url, stream=True)
        if r.status_code == 200:
            with open(self.file_name, 'wb') as f:
                r.raw.decode_content = True
                shutil.copyfileobj(r.raw, f)



    def show_image(self, url): #ダウンロードした画像の表示
        self.download_img(url) #Google画像検索から画像をダウンロード
        img = cv2.imread(self.file_name) #画像の読み込み
        zoom_rate = 2.5 #拡大率
        image = cv2.resize(img, dsize=None, fx = zoom_rate, fy = zoom_rate) #画像を拡大
        cv2.imshow("image", image) #拡大した画像を表示
        cv2.moveWindow('image', 200, 550) #ウィンドウ位置の変更
        cv2.waitKey(4000) #4秒待機
        cv2.destroyAllWindows() #ウィンドウを破棄



class Publishsers(): #パブリッシャーのクラス
    def __init__(self): #コンストラクタと呼ばれる初期化のための関数（メソッド）
        self.count = 0
        #messageの型を作成
        self.message = speech_recognition_message() 
        #speech_recognition_message型のメッセージを"recognition_txt_topic"というトピックに送信するパブリッシャーの作成
        self.publisher = rospy.Publisher('recognition_txt_topic', speech_recognition_message, queue_size=10)
        self.rate = rospy.Rate(0.3) #1秒間に0.3回データを送信する
        self.number = 0



    def explain(self): #説明
            mc = Mecab()
            Text_list, search_word_list = mc.mecab_main(self.count)
            return Text_list, search_word_list



    def make_msg(self): #送信するメッセージの作成
        Text_list, search_word_list = self.explain()
        if len(Text_list) >= self.number:
            self.message.Text = "{}".format(Text_list[self.number])
        self.message.count = self.count
        return Text_list, search_word_list



    def send_msg(self): #メッセージを送信
        Text_list, search_word_list = self.make_msg() #送信するメッセージの作成
        for self.number in range(len(Text_list)):
            self.message.Text = "{}".format(Text_list[self.number])
            self.publisher.publish(self.message) #作成したメッセージの送信
            print(self.message.Text)
            if (len(search_word_list) >= self.number) and (self.number >= 1):
                gi = Google_image() #クラスのインスタンス生成
                link = gi.create_url(search_word_list[self.number-1]) #Google画像検索のURLを生成
                gi.show_image(link) #ダウンロードした画像の表示
            self.rate.sleep()
        #rospy.loginfo("送信：message = %s, count = %d" %(self.message.Text, self.message.count))



def main(): #メイン関数
    #初期化し、ノードの名前を設定
    rospy.init_node('explain_gogen_node', anonymous=True)
    #クラスのインスタンス作成（クラス内の関数や変数を使えるようにする）
    pub = Publishsers()
    
    while not rospy.is_shutdown(): #Ctrl + Cが押されるまで繰り返す
        if len(words) > pub.count:
            pub.rate.sleep()
            pub.send_msg() #メッセージを送信
            pub.rate.sleep()
            pub.count += 1
        else:
            break



if __name__ == '__main__':
    try:
        main() #メイン関数の実行
    except rospy.ROSInterruptException: pass