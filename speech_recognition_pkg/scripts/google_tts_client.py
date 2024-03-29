#!/usr/bin/env python
# -*- coding: utf-8 -*-
#上記2行は必須構文のため、コメント文だと思って削除しないこと
#Python2.7用プログラム

import rospy
import sys
from speech_recognition_pkg.srv import play_end_check_service, google_tts_service # サービスファイルの読み込み（from パッケージ名.srv import 拡張子なしサービスファイル名）
import alkana

words_list = ["contour", "conceal"]
meanings_list = ["輪郭", "覆う"]



class Google_TTS_Client():  # クライアントのクラス
    def __init__(self):  # コンストラクタと呼ばれる初期化のための関数（メソッド）
        self.service_message = google_tts_service()
        self.count = 0



    def exec_alkana(self, text):
        alkana_word = alkana.get_kana(text)  # アルファベットをカタカナに変換
        if alkana_word != None:  # alkanaの辞書にそのアルファベットが存在する場合
            alkana_text = alkana_word  # カタカナに置き換える
            return alkana_text
        else:
            return text



    def make_request(self): #リクエストの作成
        word = self.exec_alkana(words_list[self.count])
        self.service_message.Text = "{}は「{}」という意味です。".format(word, meanings_list[self.count])



    def google_tts_service_request(self): #Google TTSサービスのリクエスト
        rospy.wait_for_service('google_tts_service')  # サービスが使えるようになるまで待機
        try:
            for self.count in range(len(words_list)):
                self.make_request() #リクエストの作成
                self.client = rospy.ServiceProxy(
                'google_tts_service', google_tts_service)  # クライアント側で使用するサービスの定義
                print("\n\n{}は「{}」という意味です。\n\n".format(words_list[self.count], meanings_list[self.count]))
            #「戻り値 = self.client(引数)」。クライアントがsrvファイルで定義した引数（srvファイル内「---」の上側）を別ファイルのサーバーにリクエストし、サーバーからの返り値（srvファイル内「---」の下側）をresponseに代入
                response = self.client(self.service_message.Text)

        except rospy.ServiceException:
            rospy.loginfo("")



def main(): #メイン関数
    #初期化し、ノードの名前を設定
    rospy.init_node('explain_node', anonymous=True)
    #クラスのインスタンス作成（クラス内の関数や変数を使えるようにする）
    gtc = Google_TTS_Client()
    gtc.google_tts_service_request()
    rospy.spin()



if __name__ == '__main__':
    try:
        main() #メイン関数の実行
    except rospy.ROSInterruptException: pass