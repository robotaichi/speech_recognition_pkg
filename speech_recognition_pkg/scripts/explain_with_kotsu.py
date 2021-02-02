#!/usr/bin/env python
# -*- coding: utf-8 -*-
#上記2行は必須構文のため、コメント文だと思って削除しないこと
#Python2.7用プログラム

#ROS関係ライブラリ
import rospy #ROSをpythonで使用するのに必要
import actionlib #アクション通信に使用
from speech_recognition_pkg.srv import play_end_check_service, speech_recognition_service, openjtalk_service, google_image_service # サービスファイルの読み込み（from パッケージ名.srv import 拡張子なしサービスファイル名）
from face_recognition_pkg.msg import realsense_actionAction, realsense_actionResult, realsense_actionGoal, check_finish_actionAction, check_finish_actionResult #メッセージファイルの読み込み（from パッケージ名.msg import 拡張子なしアクションメッセージファイル名）。actionフォルダで定義した「アクションファイル名.action」ファイルを作成し、catkin_makeすると、「アクションファイル名Action.msg」、「アクションファイル名Feedback.msg」、「アクションファイル名ActionFeedback.msg」、「アクションファイル名Goal.msg」、「アクションファイル名ActionGoal.msg」、「アクションファイル名Result.msg」、「アクションファイル名ActionResult.msg」が生成される。生成されたアクションメッセージファイルは、「ls /home/limlab/catkin_ws/devel/share/パッケージ名/msg」コマンドで確認できる。アクションサーバ側は、「アクションファイル名Action.msg」、「アクションファイル名Result.msg」（途中経過が必要な場合は、「アクションファイル名Feedback.msg」）をインポートする。「アクションファイル名Goal.msg」は、アクションクライアントからリクエストがあった場合に呼び出されるコールバック関数の引数として取得できるため、アクションサーバ側は必要ない。アクションクライアント側は、「アクションファイル名Action.msg」、「アクションファイル名Result.msg」、「アクションファイル名Goal.msg」（途中経過が必要な場合は、「アクションファイル名Feedback.msg」）をインポートする。
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal #メッセージファイルの読み込み（from パッケージ名.msg import 拡張子なしアクションメッセージファイル名
from trajectory_msgs.msg import JointTrajectoryPoint #メッセージファイルの読み込み（from パッケージ名.msg import 拡張子なしアクションメッセージファイル名
import math #ラジアンを扱うのに必要
import speech_recognition as sr #音声認識に使用
import sys #プログラムの終了に必要
import MeCab #形態素解析に使用
import sqlite3 #日本語WordNetに使用
import os #ファイルパスを扱うのに必要
import ssl #Google画像検索に使用
ssl._create_default_https_context = ssl._create_unverified_context #Google画像検索に使用
import cv2 #OpenCVを使用
import alkana  # アルファベットをカタカナに変換するライブラリ

#既にファイルが存在する場合
if os.path.exists("/home/limlab/catkin_ws/src/speech_recognition_pkg/google_image_url.txt"): 
    os.remove("/home/limlab/catkin_ws/src/speech_recognition_pkg/google_image_url.txt") #ファイルの削除

#出題する英単語とその意味の設定
words = ["contour", "implement", "instance", "artificial", "distribute", "unlike", "abstract", "resemble", "investigate", "conclusion"] #単語リスト
meanings = ["輪郭", "実行する", "実例", "人工の", "分配する", "ちがって", "抽象的な", "似ている", "調べる", "結論"] #意味リスト

#デバッグ用
#words = ["area", "artificial"] #単語リスト
#meanings = ["面積", "人工の"] #意味リスト

#もう一度説明が必要だと判定する音声認識結果のリスト
recognized_text_list = ["いいえ", "いえ", "うーん", "ううん", "えーと", "あり", "ませ", "わから", "いや", "ノー", "NO", "えっ", "1", "一", "回", "度", "いちど", "かい", "教え", "お願い", "わかん", "ない", "まだ", "ちょっと", "忘れ"]

#変数設定
Language = 'ja-JP' #音声認識の対象言語を設定
r = sr.Recognizer()
mic = sr.Microphone() #マイクの設定
element_group_number = 2 #配列を()ごとにまとめる際の()に含まれる要素数



class Mecab(): #Mecabのクラス
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



    def with_blank(self, node): #空白ありのリスト生成
        while node:
            self.list_with_blank1.append(node.surface)
            word_class = node.feature.split(',') #word_classの構造：品詞, 品詞細分類1, 品詞細分類2, 品詞細分類3, 活用形, 活用型, 原形, 読み, 発音
            self.list_with_blank2.append(word_class[6].replace("*", "")) #「*」を消去
            #hinshi = self.node.feature.split(",")[0]
            node = node.next
        return self.list_with_blank1, self.list_with_blank2



    def without_blank(self, list_with_blank1, list_with_blank2): #空白なしのリスト生成
        for element in list_with_blank1:
            if element != '': #形態素がある（要素が存在する）場合
                self.list_without_blank1.append(element)
        for element in list_with_blank2:
            if element != '': #語源がある（要素が存在する）場合
                self.list_without_blank2.append(element)
        return self.list_without_blank1, self.list_without_blank2



    def combine(self, list_without_blank1, list_without_blank2): #単語と語源のリストを結合
        for i in range(len(list_without_blank1)): #単語と語源の配列を連結
            self.output.append(list_without_blank1[i])
            self.output.append(list_without_blank2[i])
        self.output = zip(*[iter(self.output)]*element_group_number) #element_group_numberごとに配列の中身を()でまとめる



    def execute_node(self, node): #ノードの実行（形態素解析）
        self.word_with_blank, self.gogen_with_blank = self.with_blank(node) #空白ありのリスト生成
        self.word_without_blank, self.gogen_without_blank = self.without_blank(self.word_with_blank, self.gogen_with_blank) #空白なしのリスト生成
        if len(self.gogen_without_blank) == 0: #語源がない（配列の要素が全て空の）場合
            self.gogen_without_blank.append(None) #Noneを語源リストに追加
        self.combine(self.word_without_blank, self.gogen_without_blank) #単語と語源のリストを結合



    def synonym_to_katakana(self, synonym): #アルファベットに変換
        alkana_word = alkana.get_kana(synonym)  # アルファベットをカタカナに変換
        if alkana_word != None:  # alkanaの辞書にそのアルファベットが存在する場合
            synonym_kana = alkana_word  # speak_listの要素をカタカナに置き換える
        else:
            synonym_kana = synonym
        return synonym_kana



    def word_to_katakana(self, number): #アルファベットに変換
        alkana_word = alkana.get_kana(words[number])  # アルファベットをカタカナに変換
        if alkana_word != None:  # alkanaの辞書にそのアルファベットが存在する場合
            text = alkana_word  # speak_listの要素をカタカナに置き換える
        else:
            text = words[number]
        return text



    def etymology_to_katakana(self, etymology): #アルファベットに変換
        alkana_word = alkana.get_kana(etymology)  # アルファベットをカタカナに変換
        if alkana_word != None:  # alkanaの辞書にそのアルファベットが存在する場合
            text = alkana_word  # speak_listの要素をカタカナに置き換える
        else:
            text = etymology
        return text



    def make_explanation(self, number, text): #説明文の作成
        text1 = "{}は「{}」と発音します\n".format(words[number], text)
        self.Text_list.append(text1)
        wn = Wordnet()
        synonym = wn.search_similar_words(self.mecab_for_wordnet(meanings[number]), words[number]) #日本語WordNet用の形態素解析
        if synonym == None: #WordNetに類義語が存在しない場合
            text2 = "{}の意味は「{}」です\n".format(words[number], meanings[number])
        else:
            synonym_kana = self.synonym_to_katakana(synonym)
            text2 = "{}の意味は「{}」で、類義語は{}「{}」です\n".format(words[number], meanings[number], synonym, synonym_kana)
        self.Text_list.append(text2)
        etymology_list = [] #語源リスト
        etymology_kana_list = [] #語源のカタカナリスト
        etymology_text_list = [] #語源のテキストリスト
        etymology_meaning_list = [] #語源の意味リスト
        etymology_meaning_set_list = []
        etymology_meaning_full_list = []
        search_word_list = [] #Googleイメージ検索する単語のリスト

        for i in range(len(self.output)):
            if self.output[i][1] != None: #語源がある（要素が存在する）場合
                etymology_list.append(self.output[i][0]) #語源リストに追加
                etymology_kana = self.etymology_to_katakana(self.output[i][0]) #アルファベットに変換
                etymology_kana_list.append(etymology_kana) #語源のカタカナリストに追加
                etymology_meaning_list.append(self.output[i][1]) #語源の意味リストに追加

        for i in range(len(self.output)):
            if self.output[i][1] != None: #語源がある（要素が存在する）場合
                etymology_text_list.append(etymology_list[i]) #語源のテキストリストに追加
                etymology_text_list.append("「{}」".format(etymology_kana_list[i])) #語源のテキストリストに追加
                if len(self.output)-1 > i: #語源の要素が最後でない場合
                    etymology_text_list.append("と") #「と」というつなぎ言葉を追加する
        if len(etymology_list) != 0: #語源がある（要素が存在する）場合
            etymology_text = "".join(etymology_text_list) #1つのテキスト化
            text3 = "{}は、".format(words[number]) + etymology_text + "に分けることができます\n"
            self.Text_list.append(text3)

        for i in range(len(self.output)):
            if self.output[i][1] != None: #語源がある（要素が存在する）場合
                etymology_meaning_set_list.append("{}が「{}」".format(self.output[i][0], self.output[i][1]))
                if len(self.output)-1 > i: #語源の要素が最後でない場合
                    etymology_meaning_set_list.append("、") #「、」の区切りを追加する
        if len(etymology_list) != 0: #語源がある（要素が存在する）場合
            etymology_meaning_set = "".join(etymology_meaning_set_list) #1つのテキスト化
            text4 = "それぞれの語源の意味は、" + etymology_meaning_set + "です\n"
            self.Text_list.append(text4)

        for i in range(len(self.output)):
            if self.output[i][1] != None: #語源がある（要素が存在する）場合
                etymology_meaning_full_list.append(self.output[i][1])

        if "否定" in etymology_meaning_full_list: #「否定」が含まれている場合
            etymology_meaning_full_list.append("の")
            etymology_meaning_full_list.append(etymology_meaning_full_list.pop(0)) #先頭(0番目)の文字「否定」を削除し(pop)、一番後ろに追加(append)

        if "形容詞化" in etymology_meaning_full_list: #「形容詞化」が含まれている場合
            etymology_meaning_full_list.append("の")
            etymology_meaning_full_list.append(etymology_meaning_full_list.pop(len(etymology_meaning_full_list)-2)) #要素数-から2を引いた位置にある文字「形容詞化」を削除し(pop)、一番後ろに追加(append)

        if "動詞化" in etymology_meaning_full_list: #「動詞化」が含まれている場合
            etymology_meaning_full_list.append("の")
            etymology_meaning_full_list.append(etymology_meaning_full_list.pop(len(etymology_meaning_full_list)-2)) #要素数-から2を引いた位置にある文字「動詞化」を削除し(pop)、一番後ろに追加(append)
        
        if "名詞化" in etymology_meaning_full_list: #「動詞化」が含まれている場合
            etymology_meaning_full_list.append("の")
            etymology_meaning_full_list.append(etymology_meaning_full_list.pop(len(etymology_meaning_full_list)-2)) #要素数-から2を引いた位置にある文字「動詞化」を削除し(pop)、一番後ろに追加(append)

        if len(etymology_list) != 0: #語源がある（要素が存在する）場合
            _text5 = "".join(etymology_meaning_full_list)
            text5 = "まとめると" + "「{}」\n".format(_text5)
            self.Text_list.append(text5)

        for i in range(len(self.output)):
            if (self.output[i][1] != None) and (self.first == True): #語源がある（要素が存在する）場合
                text6 = "「{}」から、{}は「{}」という意味になります\n".format(_text5, words[number], meanings[number])
                self.first = False
            elif self.output[i][1] == None: #語源がない（配列の要素が全て空の）場合
                text6 = "もう一度確認で、{}は「{}」という意味です\n".format(words[number], meanings[number])
        self.Text_list.append(text6)
        return self.Text_list, search_word_list, etymology_list, etymology_meaning_list, etymology_kana_list, synonym



    def mecab_for_wordnet(self, text): #日本語WordNet用の形態素解析
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



    def mecab_wakati(self, voice_recognition_response): #認識した音声の分かち書き
        wakati_text_list = []
        mecabTagger = MeCab.Tagger("-Owakati \ -d /usr/local/lib/mecab/dic/ipadic \ -u /usr/local/lib/mecab/dic/userdic/gogen.dic") # -d(--dicdir)：使用するシステム辞書を指定 -u(--userdic):ユーザ辞書を指定
        wakati_text = mecabTagger.parse(voice_recognition_response).split() 
        for i in range(len(wakati_text)):
            wakati_text_list.append(wakati_text[i])
        return wakati_text_list



    def mecab_main(self, count): #Mecabのメイン処理
        mecabTagger = MeCab.Tagger("-Ochasen \ -d /usr/local/lib/mecab/dic/ipadic \ -u /usr/local/lib/mecab/dic/userdic/gogen.dic") # -d(--dicdir)：使用するシステム辞書を指定 -u(--userdic):ユーザ辞書を指定
        mecabTagger.parse("")  # Unicode Decode Errorの回避
        node = mecabTagger.parseToNode(words[count])
        self.execute_node(node)
        text = self.word_to_katakana(count)
        Text_list, search_word_list, etymology_list, etymology_meaning_list, etymology_kana_list, synonym = self.make_explanation(count, text) #説明文の作成
        return Text_list, search_word_list, etymology_list, etymology_meaning_list, etymology_kana_list, synonym



class Wordnet(): #日本語WordNetのクラス
    def __init__(self):
        self.conn = sqlite3.connect("/home/limlab/Programs/wordnet/wnjpn.db")



    def search_similar_words(self, search_word, word): # 特定の単語を入力とした時に、類義語を検索する関数
        # 問い合わせしたい単語がWordnetに存在するか確認する
        cur = self.conn.execute("select wordid from word where lemma='%s'" % search_word)
        word_id = 99999999  #一時的なID

        for row in cur:
            word_id = row[0]

        if word_id==99999999: # Wordnetに存在する語であるかの判定
            return None

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
        for i in range(len(synonyms)):
            if (synonyms[i] != word) and (not "_" in synonyms[i]):
                return synonyms[i]



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



class NeckPitch(object): #Sciurus17のお辞儀のクラス
    def __init__(self):
        self.client = actionlib.SimpleActionClient("/sciurus17/controller3/neck_controller/follow_joint_trajectory", FollowJointTrajectoryAction) #アクションサーバを作成
        self.client.wait_for_server(rospy.Duration(5.0)) #アクションサーバとの接続待機、5秒でタイムアウト
        if not self.client.wait_for_server(rospy.Duration(5.0)): #アクションサーバとの接続ができなかった場合
            rospy.logerr("Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)
        self.yaw_angle = 0.0
        self.pitch_angle = 0.0



    def set_angle(self, yaw_angle, pitch_angle): #関節の設定（お辞儀動作）
        goal = FollowJointTrajectoryGoal() #アクション目標（Goal）のインスタンス生成
        goal.trajectory.joint_names = ["neck_yaw_joint", "neck_pitch_joint"]
        yawpoint = JointTrajectoryPoint() #アクションメッセージのインスタンス生成
        self.yaw_angle = yaw_angle
        self.pitch_angle = pitch_angle
        yawpoint.positions.append(self.yaw_angle) #リストに追加
        yawpoint.positions.append(self.pitch_angle) #リストに追加
        yawpoint.time_from_start = rospy.Duration(nsecs=1)
        goal.trajectory.points.append(yawpoint) #リストに追加
        self.client.send_goal(goal) #アクションサーバにアクション目標（Goal）を送信。ここでは、定義したアクション目標（Goal）のインスタンスを引数に指定すること。アクション目標（Goal）を送信した時点でアクションサーバが起動（動作開始）し、アクションクライアントも次の処理を実行する（並列処理になる）
        self.client.wait_for_result(rospy.Duration(0.1)) # 結果が返ってくるまで0.1秒待機。ここで処理が一時停止する
        return self.client.get_result() #アクション結果を取得し、返す



    def main(self): #Sciurus17のお辞儀に必要なパラメータを代入
        self.set_angle(math.radians(0.0), math.radians(-60.0)) #下向き
        rospy.sleep(1.0)
        self.set_angle(math.radians(0.0), math.radians(60.0)) #上向き
        rospy.sleep(1.0)
        self.set_angle(math.radians(0.0), math.radians(0.0)) #ホームポジション（正面）
        rospy.sleep(1.0)



class OpenJTalk_Client():  # クライアントのクラス
    def __init__(self):  # コンストラクタと呼ばれる初期化のための関数（メソッド）
        self.rate = rospy.Rate(1)  # 1秒間に1回データを受信する
        self.openjtalk_service_message = openjtalk_service()
        self.count = 0 #現在の単語のカウント
        self.number = 0 #説明文のパート番号



    def introduction(self): #イントロダクション
        self.openjtalk_service_message.openjtalk_request = "わたしはシューラスセブンティーンです"
        print("\n\n{}\n\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
        self.openjtalk_service_message.openjtalk_request = "今から説明を始めます"
        print("\n\n{}\n\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
        np = NeckPitch()
        np.main()



    def exec_mecab(self): #Mecabによる形態素解析の実行
            mc = Mecab()
            Text_list, search_word_list, etymology_list, etymology_meaning_list, etymology_kana_list, synonym = mc.mecab_main(self.count)
            return Text_list, search_word_list, etymology_list, etymology_meaning_list, etymology_kana_list, synonym



    def google_image(self, search_word, text): #Google画像検索の処理
        gic = Google_Image_Client()
        gic.google_image_service_request(search_word, text)



    def make_request(self): #リクエストの作成
        Text_list, search_word_list, etymology_list, etymology_meaning_list, etymology_kana_list, synonym = self.exec_mecab() #Mecabによる形態素解析の実行

        if len(Text_list) >= self.number:
            self.openjtalk_service_message.openjtalk_request = "{}".format(Text_list[self.number])
        return Text_list, search_word_list, etymology_list, etymology_meaning_list, etymology_kana_list, synonym



    def realsense_action_request(self): #リアルセンスアクションのリクエスト
            rs = Realsense_Action_Client() #クラスのインスタンス生成
            rs.make_goal() #アクション目標（Goal）の作成
            return rs



    def ask_question(self): #わかったかどうかの質問
        self.openjtalk_service_message.openjtalk_request = "今の説明で分かりましたか？\n"
        print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
        vrc = Speech_Recognition_Client() #クラスのインスタンス生成
        voice_recognition_response = vrc.speech_recognition_service_request() #音声認識サービスのリクエスト
        return voice_recognition_response



    def check_pronunciation(self): #発音チェック
        self.openjtalk_service_message.openjtalk_request = "発音してください\n"
        print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
        vrc = Speech_Recognition_Client() #クラスのインスタンス生成
        voice_recognition_response = vrc.speech_recognition_service_request() #音声認識サービスのリクエスト



    def check_divited_etymology(self, etymology_list, etymology_kana_list, count): #語源の分割を覚えたかどうか確認
        self.openjtalk_service_message.openjtalk_request = "確認します\n"
        print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
        self.openjtalk_service_message.openjtalk_request = "{}はいくつの語源に分けることができますか？\n".format(words[count])
        print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
        vrc = Speech_Recognition_Client() #クラスのインスタンス生成
        voice_recognition_response = vrc.speech_recognition_service_request() #音声認識サービスのリクエスト
        wakati_text_list = self.voice_recognition_response_mecab(voice_recognition_response) #認識した音声の形態素解析
        i = 1
        for wakati_text in wakati_text_list:
            if str(len(etymology_list)) in wakati_text:
                self.openjtalk_service_message.openjtalk_request = "正解です\n"
                print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
                self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
                break
            else:
                if i == len(wakati_text_list):
                    self.openjtalk_service_message.openjtalk_request = "残念\n"
                    print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
                    self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
                    break
                else:
                    i += 1
        self.openjtalk_service_message.openjtalk_request = "{}つですね\n".format(len(etymology_list))
        print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト

        number = 0
        while True: #語源リストに語源がある間繰り返す
            self.openjtalk_service_message.openjtalk_request = "{}の語源の{}つめは？\n".format(words[count], number + 1)
            print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
            self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
            vrc = Speech_Recognition_Client() #クラスのインスタンス生成
            voice_recognition_response = vrc.speech_recognition_service_request() #音声認識サービスのリクエスト
            self.openjtalk_service_message.openjtalk_request = "{}ですね\n".format(etymology_list[number])
            print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
            self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
            self.openjtalk_service_message.openjtalk_request = "{}に関連した画像を表示します\n".format(etymology_list[number])
            print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
            self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
            self.google_image(etymology_list[number], self.openjtalk_service_message.openjtalk_request) #Google画像検索の処理
            wakati_text_list = self.voice_recognition_response_mecab(voice_recognition_response) #認識した音声の形態素解析
            for wakati_text in wakati_text_list:
                if wakati_text in recognized_text_list: #もう一度説明が必要だと判定するリストに音声認識結果が含まれている場合:
                    self.openjtalk_service_message.openjtalk_request = "もう一度確認します\n"
                    print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
                    self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
                    if number > 0:
                        number = number - 1 #1つ前の値に戻る
                    else: #countが0の場合
                        number = -1 #+1したときに0となるように
                    break
            number += 1

            if number == len(etymology_list): #語源がなくなった場合
                text_list = []
                for i in range(len(etymology_list)):
                    text_list.append(etymology_list[i])
                    text_list.append("「{}」".format(etymology_kana_list[i]))
                    if len(etymology_list)-1 > i: #語源が最後でない場合
                        text_list.append("と") #「と」というつなぎ言葉を追加
                text = "".join(text_list) #リストを文字列化
                self.openjtalk_service_message.openjtalk_request = "{}は、".format(words[count]) + text + "に分けることができます。OKですか？\n"
                print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
                self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
                vrc = Speech_Recognition_Client() #クラスのインスタンス生成
                voice_recognition_response = vrc.speech_recognition_service_request() #音声認識サービスのリクエスト
                wakati_text_list = self.voice_recognition_response_mecab(voice_recognition_response) #認識した音声の形態素解析

                again = False #againの初期化
                for wakati_text in wakati_text_list:
                    if wakati_text in recognized_text_list: #もう一度説明が必要だと判定するリストに音声認識結果が含まれている場合
                        again = True #againのTrueフラグを立てる
                        break
                if again: #againがTrueの場合
                    number = 0
                    self.openjtalk_service_message.openjtalk_request = "わかりました。もう一度確認します\n"
                    print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
                    self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
                    continue #whileループの最初に戻る
                else: #againがFalseの場合
                    break



    def check_etymology_meaning(self, etymology_list, etymology_meaning_list, count): #語源の日本語の意味を覚えたかどうか確認
        self.openjtalk_service_message.openjtalk_request = "確認します\n"
        print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト

        number = 0
        while True: #語源リストに語源がある間繰り返す
            self.openjtalk_service_message.openjtalk_request = "{}の日本語の意味は?\n".format(etymology_list[number])
            print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
            self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
            vrc = Speech_Recognition_Client() #クラスのインスタンス生成
            voice_recognition_response = vrc.speech_recognition_service_request() #音声認識サービスのリクエスト
            self.openjtalk_service_message.openjtalk_request = "「{}」ですね\n".format(etymology_meaning_list[number])
            print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
            self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
            self.openjtalk_service_message.openjtalk_request = "「{}」に関連した画像を表示します\n".format(etymology_meaning_list[number])
            print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
            self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
            self.google_image(etymology_meaning_list[number], self.openjtalk_service_message.openjtalk_request) #Google画像検索の処理
            wakati_text_list = self.voice_recognition_response_mecab(voice_recognition_response) #認識した音声の形態素解析
            for wakati_text in wakati_text_list:
                if wakati_text in recognized_text_list: #もう一度説明が必要だと判定するリストに音声認識結果が含まれている場合:
                    self.openjtalk_service_message.openjtalk_request = "もう一度確認します\n"
                    print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
                    self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
                    if number > 0:
                        number = number - 1 #1つ前の値に戻る
                    else: #countが0の場合
                        number = -1 #+1したときに0となるように
                    break
            number += 1

            if number == len(etymology_list): #語源がなくなった場合
                text_list = []
                for i in range(len(etymology_list)):
                    text_list.append(etymology_list[i])
                    text_list.append("が")
                    text_list.append("「{}」".format(etymology_meaning_list[i]))
                    if len(etymology_list)-1 > i: #語源が最後でない場合
                        text_list.append("、") #「、」の区切りを追加
                text = "".join(text_list)
                self.openjtalk_service_message.openjtalk_request = "{}の語源の意味は、".format(words[count]) + text + "です。OKですか？\n"
                print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
                self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
                vrc = Speech_Recognition_Client() #クラスのインスタンス生成
                voice_recognition_response = vrc.speech_recognition_service_request() #音声認識サービスのリクエスト
                wakati_text_list = self.voice_recognition_response_mecab(voice_recognition_response) #認識した音声の形態素解析

                again = False #againの初期化
                for wakati_text in wakati_text_list:
                    if wakati_text in recognized_text_list: #もう一度説明が必要だと判定するリストに音声認識結果が含まれている場合
                        again = True #againのTrueフラグを立てる
                        break
                if again: #againがTrueの場合
                    number = 0
                    self.openjtalk_service_message.openjtalk_request = "わかりました。もう一度確認します\n"
                    print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
                    self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
                    continue #whileループの最初に戻る
                else: #againがFalseの場合
                    break



    def check_word_meaning(self, count):
        self.openjtalk_service_message.openjtalk_request = "確認します\n"
        print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト

        while True: #語源リストに語源がある間繰り返す
            self.openjtalk_service_message.openjtalk_request = "{}の日本語の意味は？\n".format(words[count])
            print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
            self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
            vrc = Speech_Recognition_Client() #クラスのインスタンス生成
            voice_recognition_response = vrc.speech_recognition_service_request() #音声認識サービスのリクエスト
            wakati_text_list = self.voice_recognition_response_mecab(voice_recognition_response) #認識した音声の形態素解析
            self.openjtalk_service_message.openjtalk_request = "{}の意味は「{}」ですね\n".format(words[count], meanings[count])
            print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
            self.openjtalk_service_request() #OpenJTalkサービスのリクエスト

            again = False #againの初期化
            for wakati_text in wakati_text_list:
                if wakati_text in recognized_text_list: #もう一度説明が必要だと判定するリストに音声認識結果が含まれている場合:
                    again = True #againのTrueフラグを立てる
                    break
            if again: #againがTrueの場合
                self.openjtalk_service_message.openjtalk_request = "わかりました。もう一度確認します\n"
                print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
                self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
                continue #whileループの最初に戻る
            else: #againがFalseの場合
                break



    def explain(self, Text_list, search_word_list, etymology_list, etymology_meaning_list, etymology_kana_list, synonym, count): #説明
        for self.number in range(len(Text_list)): #説明文がある間繰り返す
            self.openjtalk_service_message.openjtalk_request = "{}".format(Text_list[self.number]) #説明文の1パートをOpenJTalkリクエストメッセージに代入
            print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
            self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
            if "発音" in Text_list[self.number]: #self.number=0の場合
                self.check_pronunciation() #発音チェック
            if "分ける" in Text_list[self.number]: #self.number=2の場合
                self.check_divited_etymology(etymology_list, etymology_kana_list, count) #語源の分割を覚えたかどうか確認
            if "それぞれ" in Text_list[self.number]: #self.number=3の場合
                self.check_etymology_meaning(etymology_list, etymology_meaning_list, count) #語源の日本語の意味を覚えたかどうか確認
            if "まとめると" in Text_list[self.number]: #self.number=4の場合
                self.repeat_summary()
            if "意味になります" in Text_list[self.number]: #self.number=5の場合
                self.check_word_meaning(count)
            # if (len(search_word_list) >= self.number) and (self.number >= 3): #説明文のパートが存在する、かつ説明文の4（0,1,2,3の4つ）パート目から
            #     self.google_image(Text_list, search_word_list) #Google画像検索の処理



    def repeat_summary(self):
        self.openjtalk_service_message.openjtalk_request = "リピートしてください\n"
        print("\n\n{}\n".format(self.openjtalk_service_message.openjtalk_request))
        self.openjtalk_service_request() #OpenJTalkサービスのリクエスト
        vrc = Speech_Recognition_Client() #クラスのインスタンス生成
        voice_recognition_response = vrc.speech_recognition_service_request() #音声認識サービスのリクエスト



    def explain_again(self, Text_list, search_word_list): #もう一度説明
        self.number = 0 #説明文のパート番号
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
        np = NeckPitch()
        np.main()



    def voice_recognition_response_mecab(self, voice_recognition_response): #認識した音声の形態素解析
            mc = Mecab()
            wakati_text_list = mc.mecab_wakati(voice_recognition_response) #認識した音声の分かち書き
            return wakati_text_list



    def main_loop(self): #メインループ
        while True:
            rs = self.realsense_action_request() #リアルセンスアクションのリクエスト
            Text_list, search_word_list, etymology_list, etymology_meaning_list, etymology_kana_list, synonym = self.make_request() #送信するメッセージの作成
            self.explain(Text_list, search_word_list, etymology_list, etymology_meaning_list, etymology_kana_list, synonym, self.count) #説明
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



class Speech_Recognition_Client():  # クライアントのクラス
    def __init__(self):  # コンストラクタと呼ばれる初期化のための関数（メソッド）
        self.rate = rospy.Rate(1)  # 1秒間に1回データを受信する
        self.service_message = speech_recognition_service()



    def speech_recognition_service_request(self):  #音声認識サービスのリクエスト
        rospy.wait_for_service('speech_recognition_service')  # サービスが使えるようになるまで待機
        try:
            self.client = rospy.ServiceProxy(
                'speech_recognition_service', speech_recognition_service)  # クライアント側で使用するサービスの定義
            self.service_message.speech_recognition_request = "音声認識サービスのリクエスト"
            # 「戻り値 = self.client(引数)」。クライアントがsrvファイルで定義した引数（srvファイル内「---」の上側）を別ファイルのサーバーにリクエストし、サーバーからの返り値（srvファイル内「---」の下側）をresponseに代入
            response = self.client(self.service_message.speech_recognition_request)
            # rospy.loginfo("音声認識サービスのリクエストに成功：{}".format(response.voice_recognition_response))
            return response.speech_recognition_response

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
    rospy.init_node('explain_with_kotsu', anonymous=True)
    ojc = OpenJTalk_Client() #クラスのインスタンス作成（クラス内の関数や変数を使えるようにする）
    ojc.introduction() #イントロダクション
    while not rospy.is_shutdown(): #Ctrl + Cが押されるまで繰り返す
        if len(words) > ojc.count: #まだ説明する単語がある場合
            ojc.main_loop() #メインループ
            ojc.count += 1 #現在の単語のカウントを1増やす
            ojc.number = 0 #説明文のパート番号を初期化
        else:
            break



if __name__ == '__main__':
    try:
        main() #メイン関数の実行
    except rospy.ROSInterruptException: pass