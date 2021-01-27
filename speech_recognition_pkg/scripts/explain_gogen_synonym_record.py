#!/usr/bin/env python
# -*- coding: utf-8 -*-
#上記2行は必須構文のため、コメント文だと思って削除しないこと
#Python2.7用プログラム

#ROS関係ライブラリ
import rospy #ROSをpythonで使用するのに必要
import actionlib #アクション通信に使用
from speech_recognition_pkg.msg import speech_recognition_message #メッセージファイルの読み込み（from パッケージ名.msg import 拡張子なしメッセージファイル名）
from speech_recognition_pkg.srv import play_end_check_service, voice_recognition_service, openjtalk_service, google_image_service # サービスファイルの読み込み（from パッケージ名.srv import 拡張子なしサービスファイル名）
# from speech_recognition_pkg.srv import record_check_service # サービスファイルの読み込み（from パッケージ名.srv import 拡張子なしサービスファイル名）
from face_recognition_pkg.srv import calculate_service, face_recognition_service, realsense_service, voice_recognition_necessity_service, check_finish_service # サービスファイルの読み込み（from パッケージ名.srv import 拡張子なしサービスファイル名）
from face_recognition_pkg.msg import face_recognition_message, realsense_actionAction, realsense_actionResult, realsense_actionGoal, check_finish_actionAction, check_finish_actionResult, check_finish_actionGoal #メッセージファイルの読み込み（from パッケージ名.msg import 拡張子なしアクションメッセージファイル名）。actionフォルダで定義した「アクションファイル名.action」ファイルを作成し、catkin_makeすると、「アクションファイル名Action.msg」、「アクションファイル名Feedback.msg」、「アクションファイル名ActionFeedback.msg」、「アクションファイル名Goal.msg」、「アクションファイル名ActionGoal.msg」、「アクションファイル名Result.msg」、「アクションファイル名ActionResult.msg」が生成される。生成されたアクションメッセージファイルは、「ls /home/limlab/catkin_ws/devel/share/パッケージ名/msg」コマンドで確認できる。アクションサーバ側は、「アクションファイル名Action.msg」、「アクションファイル名Result.msg」（途中経過が必要な場合は、「アクションファイル名Feedback.msg」）をインポートする。「アクションファイル名Goal.msg」は、アクションクライアントからリクエストがあった場合に呼び出されるコールバック関数の引数として取得できるため、アクションサーバ側は必要ない。アクションクライアント側は、「アクションファイル名Action.msg」、「アクションファイル名Result.msg」、「アクションファイル名Goal.msg」（途中経過が必要な場合は、「アクションファイル名Feedback.msg」）をインポートする。

import speech_recognition as sr #音声認識に使用
import sys
import MeCab #形態素解析に使用
import sqlite3 #日本語WordNetに使用
import requests #Google画像検索に使用
import random #ランダムに画像を取得するために使用
import shutil #Google画像検索に使用
import bs4 #Google画像検索に使用
import os
import ssl #Google画像検索に使用
ssl._create_default_https_context = ssl._create_unverified_context #Google画像検索に使用
import cv2 #画像処理に使用
import time

#既にファイルが存在する場合
if os.path.exists("/home/limlab/catkin_ws/src/speech_recognition_pkg/google_image_url.txt"): 
    os.remove("/home/limlab/catkin_ws/src/speech_recognition_pkg/google_image_url.txt") #ファイルの削除

#出題する英単語とその意味の設定
words = ["contour", "implement", "instance", "artificial", "distribute", "unlike", "abstract", "resemble", "investigate", "conclusion"] #単語リスト
meanings = ["輪郭", "実行する", "実例", "人工の", "分配する", "ちがって", "抽象的な", "似ている", "調べる", "結論"] #意味リスト

#デバッグ用
# words = ["artificial", "unlike"] #単語リスト
# meanings = ["人工の", "ちがって"] #意味リスト

#もう一度説明が必要だと判定する音声認識結果のリスト
recognized_text_list = ["いいえ", "いえ", "うーん", "ううん", "えーと", "あり", "ませ", "わから", "いや", "ノー", "NO", "えっ", "1", "一", "回", "度", "いちど", "かい", "教え", "お願い", "わかん", "ない", "まだ", "ちょっと", "8"]

#変数設定
Language = 'ja-JP' #音声認識の対象言語を設定
r = sr.Recognizer()
mic = sr.Microphone() #マイクの設定
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
        if "否定" in search_word_list:
            search_word_list.append("の")
            search_word_list.append(search_word_list.pop(0)) #先頭(0番目)の文字「否定」を削除し(pop)、一番後ろに追加(append)   
        if "形容詞化" in search_word_list:
            search_word_list.append("の")
            search_word_list.append(search_word_list.pop(len(search_word_list)-2)) #要素数-から2を引いた位置にある文字「動詞化」を削除し(pop)、一番後ろに追加(append)
        if "動詞化" in search_word_list:
            search_word_list.append("の")
            search_word_list.append(search_word_list.pop(len(search_word_list)-2)) #要素数-から2を引いた位置にある文字「動詞化」を削除し(pop)、一番後ろに追加(append)      
        text3 = "".join(search_word_list)
        
        text4 = "まとめると" + "「{}」\n".format(text3)
        self.Text_list.append(text4)
        for i in range(len(self.output)):
            if (self.output[i][1] != None) and (self.first == True): #語源がある（要素が存在する）場合
                text5 = "から「{}」という意味になります\n".format(meanings[number])
                self.first = False
            elif self.output[i][1] == None: #語源がない（配列の要素が全て空の）場合
                text5 = "「{}」という意味です\n".format(meanings[number])
        self.Text_list.append(text5)
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



    def mecab_wakati(self, voice_recognition_response):
        wakati_text_list = []
        mecabTagger = MeCab.Tagger("-Owakati \ -d /usr/local/lib/mecab/dic/ipadic \ -u /usr/local/lib/mecab/dic/userdic/gogen.dic") # -d(--dicdir)：使用するシステム辞書を指定 -u(--userdic):ユーザ辞書を指定
        wakati_text = mecabTagger.parse(voice_recognition_response).split() 
        for i in range(len(wakati_text)):
            wakati_text_list.append(wakati_text[i])
            # print(unicode(wakati_text[i], "utf-8"))
        return wakati_text_list



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



class Google_Image_Client():  # クライアントのクラス
    def __init__(self):  # コンストラクタと呼ばれる初期化のための関数（メソッド）
        self.rate = rospy.Rate(1)  # 1秒間に1回データを受信する
        self.service_message = google_image_service()



    def google_image_service_request(self, search_word, text):  # サービスのリクエスト
        rospy.wait_for_service('google_image_service')  # サービスが使えるようになるまで待機
        try:
            self.client = rospy.ServiceProxy(
                'google_image_service', google_image_service)  # クライアント側で使用するサービスの定義
            self.service_message.search_word = search_word
            self.service_message.text = text
            #「戻り値 = self.client(引数)」。クライアントがsrvファイルで定義した引数（srvファイル内「---」の上側）を別ファイルのサーバーにリクエストし、サーバーからの返り値（srvファイル内「---」の下側）をresponseに代入
            response = self.client(self.service_message.search_word, self.service_message.text)
            return response.google_image_response

        except rospy.ServiceException:
            rospy.loginfo("Googleサービスのリクエストに失敗")



class Play_End_Check_Client():  # クライアントのクラス
    def __init__(self):  # コンストラクタと呼ばれる初期化のための関数（メソッド）
        self.rate = rospy.Rate(0.3)  # 1秒間に0.3回データを受信する
        self.service_message = play_end_check_service()



    def play_end_check_service_request(self):  # サービスのリクエスト
        rospy.wait_for_service('play_end_check_service')  # サービスが使えるようになるまで待機
        try:
            self.client = rospy.ServiceProxy(
                'play_end_check_service', play_end_check_service)  # クライアント側で使用するサービスの定義。サーバーからの返り値（srvファイル内「---」の下側）をresponseに代入
            self.service_message.play_end_check_request = "再生確認サービスのリクエスト"
            response = self.client(self.service_message.play_end_check_request)

        except rospy.ServiceException:
            rospy.loginfo("再生確認サービスのリクエストに失敗")



class OpenJTalk_Client():  # クライアントのクラス
    def __init__(self):  # コンストラクタと呼ばれる初期化のための関数（メソッド）
        self.rate = rospy.Rate(1)  # 1秒間に1回データを受信する
        self.openjtalk_service_message = openjtalk_service()
        self.count = 0
        self.number = 0



    def introduction(self):
        self.openjtalk_service_message.openjtalk_request = "今から説明を始めます"
        print("\n\n{}\n\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
        self.openjtalk_service_message.openjtalk_request = "英単語の意味をできるだけ覚えてください"
        print("\n\n{}\n\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト



    def exec_mecab(self): 
            mc = Mecab()
            Text_list, search_word_list = mc.mecab_main(self.count)
            return Text_list, search_word_list



    def google_image(self, Text_list, search_word_list): #Google画像検索の処理
        gic = Google_Image_Client()
        gic.google_image_service_request(search_word_list[self.number-1], Text_list[self.number])



    def make_request(self): #リクエストの作成
        Text_list, search_word_list = self.exec_mecab()
        # for i in range(len(Text_list)):
        #     print(Text_list[i])
        # print(len(Text_list), self.number)

        if len(Text_list) >= self.number:
            self.openjtalk_service_message.openjtalk_request = "{}".format(Text_list[self.number])
        return Text_list, search_word_list



    def realsense_action_request(self): #リアルセンスアクションのリクエスト
            rs = Realsense_Action_Client() #クラスのインスタンス生成
            rs.make_goal() #アクション目標（Goal）の作成
            return rs



    def ask_question(self): #わかったかどうかの質問
        self.openjtalk_service_message.openjtalk_request = "今の説明で分かりましたか？\n"
        print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
        vrc = Voice_Recognition_Client()
        voice_recognition_response = vrc.voice_recognition_service_request()
        return voice_recognition_response



    def explain(self, Text_list, search_word_list): #説明
        for self.number in range(len(Text_list)):
            self.openjtalk_service_message.openjtalk_request = "{}".format(Text_list[self.number])
            print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
            self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
            if (len(search_word_list) >= self.number) and (self.number >= 1):
                self.google_image(Text_list, search_word_list) #Google画像検索の処理



    def explain_again(self, Text_list, search_word_list): #もう一度説明
        self.number = 0
        self.openjtalk_service_message.openjtalk_request = "わかりました"
        print("\n\n{}\n\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
        self.openjtalk_service_message.openjtalk_request = "\n\nもう一度説明します"
        print("{}\n\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
        self.explain(Text_list, search_word_list) #説明



    def openjtalk_service_request(self): #OpenJTalkサービスのリクエスト
        rospy.wait_for_service('openjtalk_service')  # サービスが使えるようになるまで待機
        try:
            self.client = rospy.ServiceProxy(
                'openjtalk_service', openjtalk_service)  # クライアント側で使用するサービスの定義
            #「戻り値 = self.client(引数)」。クライアントがsrvファイルで定義した引数（srvファイル内「---」の上側）を別ファイルのサーバーにリクエストし、サーバーからの返り値（srvファイル内「---」の下側）をresponseに代入
            response = self.client(self.openjtalk_service_message.openjtalk_request)

        except rospy.ServiceException:
            rospy.loginfo("")



    def go_to_next_word(self): #次の単語に移る
        self.openjtalk_service_message.openjtalk_request = "次の単語に移ります\n"
        print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト



    def finish_explanation(self): #説明終了
        self.openjtalk_service_message.openjtalk_request = "以上で説明を終わります\n"
        print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
        self.openjtalk_service_message.openjtalk_request = "\n\nお疲れ様でした"
        print("{}\n\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト



    def voice_recognition_response_mecab(self, voice_recognition_response): #認識した音声の形態素解析
            mc = Mecab()
            wakati_text_list = mc.mecab_wakati(voice_recognition_response)
            return wakati_text_list



    def main_loop(self):
        while True:
            rs = self.realsense_action_request() #リアルセンスアクションのリクエスト
            Text_list, search_word_list = self.make_request() #送信するメッセージの作成
            self.explain(Text_list, search_word_list) #説明
            voice_recognition_necessity = rs.request_result() #アクション結果のリクエスト

            if voice_recognition_necessity: #音声認識が必要（True）な場合
                voice_recognition_response = self.ask_question() #わかったかどうかの質問
                wakati_text_list = self.voice_recognition_response_mecab(voice_recognition_response) #認識した音声の形態素解析

                for wakati_text in wakati_text_list:
                    if wakati_text in recognized_text_list: #もう一度説明が必要だと判定するリストに音声認識結果が含まれている場合
                        self.explain_again(Text_list, search_word_list) #もう一度説明
                        break
                if len(words)-1 > self.count: #まだ説明する単語がある場合
                    self.go_to_next_word() #次の単語に移る
                else: #説明する単語がもうない場合
                    self.finish_explanation() #説明終了
                break
            else: #音声認識が必要でない（False）場合
                if len(words)-1 > self.count: #まだ説明する単語がある場合
                    self.go_to_next_word() #次の単語に移る
                else: #説明する単語がもうない場合
                    self.finish_explanation() #説明終了
                break



class Voice_Recognition_Client():  # クライアントのクラス
    def __init__(self):  # コンストラクタと呼ばれる初期化のための関数（メソッド）
        self.rate = rospy.Rate(1)  # 1秒間に1回データを受信する
        self.service_message = voice_recognition_service()



    def voice_recognition_service_request(self):  # サービスのリクエスト
        rospy.wait_for_service('voice_recognition_service')  # サービスが使えるようになるまで待機
        try:
            self.client = rospy.ServiceProxy(
                'voice_recognition_service', voice_recognition_service)  # クライアント側で使用するサービスの定義
            self.service_message.voice_recognition_request = "音声認識サービスのリクエスト"
            # 「戻り値 = self.client(引数)」。クライアントがsrvファイルで定義した引数（srvファイル内「---」の上側）を別ファイルのサーバーにリクエストし、サーバーからの返り値（srvファイル内「---」の下側）をresponseに代入
            response = self.client(self.service_message.voice_recognition_request)
            # rospy.loginfo("音声認識サービスのリクエストに成功：{}".format(response.voice_recognition_response))
            return response.voice_recognition_response

        except rospy.ServiceException:
            rospy.loginfo("音声認識サービスのリクエストに失敗")



class Realsense_Action_Client():  #アクションクライアントのクラス
    def __init__(self):  # コンストラクタと呼ばれる初期化のための関数（メソッド）
        self.rate = rospy.Rate(1)  # 1秒間に1回データを受信する
        # self.realsense_service_message = realsense_service()
        self.goal = realsense_actionGoal() #アクション目標（Goal）のインスタンス生成
        self.result = realsense_actionResult() #アクション結果（Result）のインスタンス生成
        self.realsense_action_client = actionlib.SimpleActionClient('realsense_action', realsense_actionAction) #「realsense_action」という名前でrealsense_actionAction型のアクションクライアントを作成



    def make_goal(self): #アクション目標（Goal）の作成
        self.goal.action_request = "リアルセンスアクションサーバのリクエスト"
        self.action_service_request() #アクションサービスのリクエスト



    def request_result(self): #アクション結果（Result）のリクエスト
        # rospy.loginfo("アクションサーバの待機中")
        # 結果が返ってくるまで30秒待機。ここで処理が一時停止（ブロッキング）する
        self.realsense_action_client.wait_for_result(rospy.Duration(30))
        result = self.realsense_action_client.get_result() # アクションサーバの「アクションサーバ.set_succeeded(アクション結果)」によって返されたアクション結果を取得
        # rospy.loginfo("アクション結果：{}".format(result.voice_recognition_necessity))
        return result.voice_recognition_necessity



    def action_service_request(self): #アクションサービスのリクエスト
        self.realsense_action_client.wait_for_server(rospy.Duration(15)) #アクションサーバーが起動するまで待機。15秒でタイムアウト 
        try:
            self.realsense_action_client.send_goal(self.goal) #アクションサーバにアクション目標（Goal）を送信。ここでは、定義したアクション目標（Goal）のインスタンスを引数に指定すること。アクション目標（Goal）を送信した時点でアクションサーバが起動（動作開始）し、アクションクライアントも次の処理を実行する（並列処理になる）

        except rospy.ServiceException:
            rospy.loginfo("リアルセンスアクションサービスのリクエストに失敗")



class Check_Finish_Action_Server(): #アクションサーバーのクラス
    def __init__(self):
        #service_messageの型を作成
        self.result = check_finish_actionResult()
        self.check_finish_action_server = actionlib.SimpleActionServer('check_finish_action', check_finish_actionAction, execute_cb = self.action_callback, auto_start = False) #「realsense_action」という名前でrealsense_actionAction型のアクションサーバを作成
        self.check_finish_action_server.start() #アクションサーバーのスタート（アクションサーバへのリクエストがなければ、スルーして処理を続ける）



    def response_tf(self):
        self.result.check_finish_response = True
        return self.result.check_finish_response



    def action_callback(self, goal): #アクションサーバの実体（コールバック関数）
        # rospy.loginfo("\nアクションリクエストがありました：\nmessage = {}\n".format(goal.action_request)) #アクション目標（Goal）の取得
        self.result.check_finish_response = self.response_tf() #リアルセンスの終了（ブール値）をアクション結果メッセージに代入
        self.check_finish_action_server.set_succeeded(self.result) #アクション結果をアクションクライアントに返す。ここでは、定義したアクション結果（Result）のインスタンスを引数に指定すること



def main(): #メイン関数
    #初期化し、ノードの名前を設定
    rospy.init_node('explain_gogen_node', anonymous=True)
    #クラスのインスタンス作成（クラス内の関数や変数を使えるようにする）
    ojc = OpenJTalk_Client()
    ojc.introduction()
    while not rospy.is_shutdown(): #Ctrl + Cが押されるまで繰り返す
        if len(words) > ojc.count:
            ojc.main_loop()
            ojc.count += 1
            ojc.number = 0
        else:
            break



if __name__ == '__main__':
    try:
        main() #メイン関数の実行
    except rospy.ROSInterruptException: pass