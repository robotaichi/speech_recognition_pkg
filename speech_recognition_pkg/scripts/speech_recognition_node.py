#!/usr/bin/env python
# -*- coding: utf-8 -*-
#上記2行は必須構文のため、コメント文だと思って削除しないこと
#Python2.7用プログラム

import rospy
from speech_recognition_pkg.msg import speech_recognition_message #メッセージファイルの読み込み（from パッケージ名.msg import 拡張子なしメッセージファイル名）
#import pyaudio
import speech_recognition as sr


#変数設定
Language = 'ja-JP' #音声認識の対象言語を設定
#wavファイルの指定
file = '/home/limlab/catkin_ws/src/speech_recognition/output.wav'
r = sr.Recognizer()
mic = sr.Microphone() #マイクの設定



def voice_record(): #音声録音
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
    
    #with open(file, "wb") as f: #wavファイルに書き込み
    #    f.write(audio.get_wav_data())



def publisher():
    # 初期化し、ノードの名前を設定
    rospy.init_node('speech_recognition_publisher', anonymous=True)
    # speech_recognition_message型のメッセージを"recognition_txt_topic"というトピックに送信する
    pub = rospy.Publisher('recognition_txt_topic', speech_recognition_message, queue_size=10)
    rate = rospy.Rate(2) #1秒間に2回データを送信する

    count = 0

    while not rospy.is_shutdown(): #Ctrl + Cが押されるまで繰り返す
        Text = voice_record() #音声録音
        Text = u"{}".format(Text).encode("utf-8")
        rospy.loginfo("送信：message = %s, count = %d" %(Text, count))

        #送信するメッセージの作成
        msg = speech_recognition_message()
        #メッセージの変数を設定
        msg.Text = Text
        msg.count = count

        #送信
        pub.publish(msg)
        rate.sleep()
        count += 1


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException: pass
