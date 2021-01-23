#!/usr/bin/env python
# -*- coding: utf-8 -*-
#上記2行は必須構文のため、コメント文だと思って削除しないこと
#Python2.7用プログラム

import rospy
from speech_recognition_pkg.msg import speech_recognition_message #メッセージファイルの読み込み（from パッケージ名.msg import 拡張子なしメッセージファイル名）
from speech_recognition_pkg.srv import play_end_check_service, voice_recognition_service, openjtalk_service # サービスファイルの読み込み（from パッケージ名.srv import 拡張子なしサービスファイル名）
# from speech_recognition_pkg.srv import record_check_service # サービスファイルの読み込み（from パッケージ名.srv import 拡張子なしサービスファイル名）
import speech_recognition as sr
import time

#変数設定
Language = 'ja-JP' #音声認識の対象言語を設定
r = sr.Recognizer()
mic = sr.Microphone() #マイクの設定



# class Publishsers(): #パブリッシャーのクラス
#     def __init__(self): #コンストラクタと呼ばれる初期化のための関数（メソッド）
#         self.count = 0
#         #messageの型を作成
#         self.message = speech_recognition_message() 
#         #speech_recognition_message型のメッセージを"recognition_txt_topic"というトピックに送信するパブリッシャーの作成
#         self.publisher = rospy.Publisher('recognition_txt_topic', speech_recognition_message, queue_size=10)
#         self.rate = rospy.Rate(2) #1秒間に2回データを送信する



#     def make_msg(self, text): #送信するメッセージの作成
#             self.message.Text = u"{}".format(text).encode("utf-8") #voice_record関数により音声認識した結果を日本語も扱えるutf-8型にエンコード



#     def send_msg(self, text): #メッセージを送信
#         self.make_msg(text) #送信するメッセージの作成
#         for i in range(3):
#             self.publisher.publish(self.message) #作成したメッセージの送信



class OpenJTalk_Client():  # クライアントのクラス
    def __init__(self):  # コンストラクタと呼ばれる初期化のための関数（メソッド）
        self.rate = rospy.Rate(1)  # 1秒間に1回データを受信する
        self.openjtalk_service_message = openjtalk_service()



    def make_request(self, unrecognized_text): #リクエストの作成
        self.openjtalk_service_message.openjtalk_request = unrecognized_text
        # return self.openjtalk_service_message.openjtalk_request



    def openjtalk_service_request(self, unrecognized_text): #OpenJTalkサービスのリクエスト
        rospy.wait_for_service('openjtalk_service')  # サービスが使えるようになるまで待機
        try:
            self.client = rospy.ServiceProxy(
                'openjtalk_service', openjtalk_service)  # クライアント側で使用するサービスの定義
            self.make_request(unrecognized_text)
            #「戻り値 = self.client(引数)」。クライアントがsrvファイルで定義した引数（srvファイル内「---」の上側）を別ファイルのサーバーにリクエストし、サーバーからの返り値（srvファイル内「---」の下側）をresponseに代入
            response = self.client(self.openjtalk_service_message.openjtalk_request)
            # rospy.loginfo("OpenJTalkサービスのリクエストに成功：{}".format(response.openjtalk_response))
            # return response.openjtalk_response

        except rospy.ServiceException:
            rospy.loginfo("OpenJTalkサービスのリクエストに失敗")



class Voice_recognition_Server(): #サーバーのクラス
    def __init__(self):
        #service_messageの型を作成
        self.service_message = voice_recognition_service()
        self.rate = rospy.Rate(0.3) # 1秒間に0.3回データを受信する
        #self.pub = Publishsers() #パブリッシャークラスのインスタンス化(実体化)



    def voice_record(self): #音声録音
        while True:
            # pecc = Play_End_Check_Client()
            # pecc.play_end_check_service_request()
            print("\n録音開始")
            # for i in range(2):
                # self.rate.sleep()
        
            with mic as source: #マイクをソースとして開き、終了後自動的に閉じる
                r.adjust_for_ambient_noise(source) #雑音対策
                audio = r.listen(source, phrase_time_limit=3) #音声を録音
        
            print("録音終了")
            # self.rate.sleep()
        
            try: #音声を認識できた場合
                recognized_text = r.recognize_google(audio, language = Language) #音声認識
                # print(u'Transcript: {}'.format(recognized_text).encode('utf-8')) #認識した音声（テキスト）を表示
                return recognized_text
        
            #音声を認識できなかった場合
            except sr.UnknownValueError:
                unrecognized_text = unicode("音声を認識できませんでした。もう一度お話ください", 'utf-8')
                print("\n音声を認識できませんでした。もう一度お話ください\n")
                ojc = OpenJTalk_Client()
                ojc.openjtalk_service_request(unrecognized_text)
                # for i in range(4):
                #     self.rate.sleep()
                #     print("待機")

            except sr.RequestError as e:
                unrecognized_text = unicode("\nGoogle Speech Recognitionサービスからの結果をリクエストできませんでした", 'utf-8')
                print("Google Speech Recognitionサービスからの結果をリクエストできませんでした: {0}".format(e))
                ojc = OpenJTalk_Client()
                ojc.openjtalk_service_request(unrecognized_text)
                # for i in range(4):
                #     self.rate.sleep()



    def make_msg(self): #送信するメッセージの作成
            #recognized_text = self.voice_record()
            recognized_text = unicode("もういちど", "utf-8") #デバッグ用
            self.service_message.voice_recognition_response = u"{}".format(recognized_text).encode("utf-8") #voice_record関数により音声認識した結果を日本語も扱えるutf-8型にエンコード



    def success_log(self, req): #成功メッセージの表示（callback関数）
        # rospy.loginfo("\n音声認識サービスのリクエストがありました：\nmessage = {}\n".format(req.voice_recognition_request))
        # time.sleep(15)
        self.make_msg()
        return self.service_message.voice_recognition_response #srvファイルで定義した返り値をsrvに渡す。rospy.Serviceによって呼び出された関数（callback関数）内でreturnすること



    def service_response(self): #サービスの応答
        srv = rospy.Service('voice_recognition_service', voice_recognition_service, self.success_log) #サービスのリクエストがあった場合にsuccess_log関数（callback関数）を呼び出し、実行。呼び出し先の関数内で返り値をreturnする必要がある



class Play_End_Check_Client():  # クライアントのクラス
    def __init__(self):  # コンストラクタと呼ばれる初期化のための関数（メソッド）
        self.rate = rospy.Rate(0.2)  # 1秒間に0.2回データを受信する
        self.service_message = play_end_check_service()



    def play_end_check_service_request(self):  # サービスのリクエスト
        rospy.wait_for_service('play_end_check_service')  # サービスが使えるようになるまで待機
        # self.rate.sleep()
        try:
            self.client = rospy.ServiceProxy(
                'play_end_check_service', play_end_check_service)  # クライアント側で使用するサービスの定義。サーバーからの返り値（srvファイル内「---」の下側）をresponseに代入
            self.service_message.play_end_check_request = "再生確認サービスのリクエスト"
            response = self.client(self.service_message.play_end_check_request)
            # self.rate.sleep()
            rospy.loginfo("再生確認サービスのリクエストに成功：{}".format(
                response.play_end_check_response))
            # return response.play_end_check_response

        except rospy.ServiceException:
            rospy.loginfo("再生確認サービスのリクエストに失敗")



def main(): #メイン関数
    #初期化し、ノードの名前を設定
    rospy.init_node('speech_recognition_publisher', anonymous=True)
    #クラスのインスタンス作成（クラス内の関数や変数を使えるようにする）
    vrs = Voice_recognition_Server()
    vrs.service_response()
    rospy.spin()
    
    # while not rospy.is_shutdown(): #Ctrl + Cが押されるまで繰り返す
    #     pub.send_msg() #メッセージを送信
    #     pub.rate.sleep()
    #     pub.count += 1



if __name__ == '__main__':
    try:
        main() #メイン関数の実行
    except rospy.ROSInterruptException: pass