#!/usr/bin/env python
# -*- coding: utf-8 -*-
#上記2行は必須構文のため、コメント文だと思って削除しないこと
#Python2.7用プログラム

#ROS関係ライブラリ
import rospy #ROSをpythonで使用するのに必要
from speech_recognition_pkg.srv import play_end_check_service, google_image_service # サービスファイルの読み込み（from パッケージ名.srv import 拡張子なしサービスファイル名）

import requests #Google画像検索に使用
import random #ランダムに画像を取得するために使用
import bs4 #Google画像検索に使用
import ssl #Google画像検索に使用
ssl._create_default_https_context = ssl._create_unverified_context #Google画像検索に使用
import cv2 #OpenCVの使用



class Play_End_Check_Client(): #クライアントのクラス
    def __init__(self):  # コンストラクタと呼ばれる初期化のための関数（メソッド）
        self.rate = rospy.Rate(0.3)  # 1秒間に0.3回データを受信する
        self.service_message = play_end_check_service() #サービスメッセージのインスタンス生成



    def play_end_check_service_request(self): #再生終了確認サービスのリクエスト
        rospy.wait_for_service('play_end_check_service')  # サービスが使えるようになるまで待機
        try:
            self.client = rospy.ServiceProxy(
                'play_end_check_service', play_end_check_service)  # クライアント側で使用するサービスの定義。サーバーからの返り値（srvファイル内「---」の下側）をresponseに代入
            self.service_message.play_end_check_request = "再生確認サービスのリクエスト"
            response = self.client(self.service_message.play_end_check_request)

        except rospy.ServiceException:
            rospy.loginfo("再生確認サービスのリクエストに失敗")



class Google_Image_Server(): #サーバーのクラス
    def __init__(self):
        #service_messageの型を作成
        self.service_message = google_image_service() #サービスメッセージのインスタンス生成
        self.rate = rospy.Rate(0.3) # 1秒間に0.3回データを受信する
        self.file_name = "/home/limlab/catkin_ws/src/speech_recognition_pkg/scripts/download_image.png" #画像を保存するファイルの指定



    def create_url(self, data, text): #Google画像検索のURLを生成
        while True:
            response = requests.get("https://www.google.com/search?hl=jp&q=" + data + "&btnG=Google+Search&tbs=0&safe=on&tbm=isch")
            Html = response.text
            soup = bs4.BeautifulSoup(Html,'lxml')
            links = soup.find_all("img")
            link = random.choice(links).get("src")
            if link != "/images/branding/searchlogo/1x/googlelogo_desk_heirloom_color_150x55dp.gif": #Gif画像でない場合
                # print("{}\n".format(link))
                self.write_txt(link, text) #txtファイルに書き込み
                return link
            # else:
            #     print("Gif画像のURLを取得しました。再取得します")



    def write_txt(self, link, text): #txtファイルに書き込み
        with open("/home/limlab/catkin_ws/src/speech_recognition_pkg/google_image_url.txt", "a") as f: #ファイルを追記モードで開き、自動的に閉じる（with）
            f.write("{}{}\n".format(text, link))



    def download_img(self, url): #Google画像検索から画像をダウンロード
        r = requests.get(url, stream=True)
        if r.status_code == 200:
            with open(self.file_name, 'wb') as f:
                f.write(r.content)
                # r.raw.decode_content = True
                # shutil.copyfileobj(r.raw, f)



    def show_image(self, url): #ダウンロードした画像の表示
        self.download_img(url) #Google画像検索から画像をダウンロード
        img = cv2.imread(self.file_name) #画像の読み込み
        zoom_rate = 5 #拡大率
        image = cv2.resize(img, dsize=None, fx = zoom_rate, fy = zoom_rate) #画像を拡大
        cv2.imshow("image", image) #拡大した画像を表示
        cv2.moveWindow('image', 0, 200) #ウィンドウ位置の変更
        cv2.waitKey(5000) #5秒待機
        # pecc = Play_End_Check_Client() #クラスのインスタンス生成
        # pecc.play_end_check_service_request() #再生終了確認サービスのリクエスト
        cv2.destroyAllWindows() #ウィンドウを破棄


    def make_msg(self): #送信するメッセージの作成
            self.service_message.google_image_response = True



    def success_log(self, req): #成功メッセージの表示（callback関数）
        # search_word = self.mecab_wakati(req.search_word)
        link = self.create_url(req.search_word, req.text) #Google画像検索のURLを生成
        self.show_image(link) #ダウンロードした画像の表示
        self.make_msg()
        return self.service_message.google_image_response #srvファイルで定義した返り値をsrvに渡す。rospy.Serviceによって呼び出された関数（callback関数）内でreturnすること



    def service_response(self): #サービスの応答
        srv = rospy.Service('google_image_service', google_image_service, self.success_log) #サービスのリクエストがあった場合にsuccess_log関数（callback関数）を呼び出し、実行。呼び出し先の関数内で返り値をreturnする必要がある



def main(): #メイン関数
    #初期化し、ノードの名前を設定
    rospy.init_node('google_image_server', anonymous=True)
    gis = Google_Image_Server() #クラスのインスタンス生成
    gis.service_response() #サービスの応答
    rospy.spin() #終了防止



if __name__ == '__main__':
    try:
        main() #メイン関数の実行
    except rospy.ROSInterruptException: pass