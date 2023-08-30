#!/usr/bin/env python
# -- coding: utf-8 --
import openai
import rospy
import speech_recognition as sr
import os
import time
from playsound import playsound
from gtts import gTTS
from konlpy.tag import Kkma
from std_msgs.msg import String

"""
간단 수정법

인식하고 싶은 물체를 tool_list에 적는다(한글)
그 물체에 대응되는 영어 단어를 같은 위치에 적는다

"""



tool_list = ["그라인더", "니퍼", "가위", "자", "해머", "망치", "플라이어"]
en_tool_list = ["grinder","nipper", "scissors", "ruler", "hammer", "hammer", "pliers"]
bring_list = ["갖","가져다주","갖다주","가지"]
kkma=Kkma()
file_name='sample.mp3'
openai.api_key = "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" # API Key
r = sr.Recognizer()
m = sr.Microphone()
turn_off_flag=False
recog_flag=True
rospy.init_node('listener')
ans_pub=rospy.Publisher("tool_list",String,queue_size=1)
ex_answer=''

def callback(r, audio):
    global turn_off_flag, recog_flag, ex_answer
    text=''
    try:
        text = r.recognize_google(audio, language='ko')
        candidates = r.recognize_google(audio, language='ko', show_all=True)

        full_text = ''
        for text in candidates['alternative']:
            full_text = full_text + '. ' + text['transcript']
        text = text['transcript']
        print("[사용자] " + text)

        if '그만' in full_text:
            turn_off_flag=True
            print("[자바스] 장치를 종료합니다")
            speaker("장치를 종료합니다")
            
        else:
            order_flag, spoken_tool=sentence_analysis(full_text)
            if order_flag is 0:
                answer=' '.join(spoken_tool)
                answer=answer+'를 가져올게요'
                print("[자바스] "+answer)
                en_tool_answer=[]
                for i in range(len(tool_list)):
                    if tool_list[i] in spoken_tool:
                        en_tool_answer.append(en_tool_list[i])

                words_str = ' '.join(en_tool_answer)
                ans_pub.publish(words_str)
                speaker(answer)

            elif order_flag is 1:
                answer='그 물건은 가져올 수 없어요'
                print("[자바스] "+answer)
                speaker(answer)

            else:
                # API 요청 및 응답 받기
                text= text + '3줄 이하로 간결하게 설명해줘'
                completion = openai.ChatCompletion.create(
                    model="gpt-3.5-turbo",
                    messages=[{"role": "user", "content": text}]
                )

                # API 응답에서 답변 텍스트 추출
                answer = completion.choices[0].message['content']

                print("[자바스] "+answer)
                speaker(answer)
        recog_flag=True    
        ex_answer=answer
        
    except sr.UnknownValueError:
        recog_flag=False
    except sr.RequestError as e:
        print(f"[자바스] 서버 연결에 실패하였습니다 : {e}")

def speaker(text):
    tts_ko=gTTS(text=text,lang='ko')
    tts_ko.save(file_name)
    playsound(file_name)
    if os.path.exists(file_name):
        os.remove(file_name)

def sentence_analysis(sentence):
    '''문장에서 도구와 가져오라는 명령이 포함되면 Ture를 반환한다.'''
    tool_flag=False
    bring_flag=0
    pos_tags = kkma.pos(sentence)
    spoken_tool=[]
    #품사와 함께 반환

    vv_words = [word for word, pos in pos_tags if pos == 'VV']

    for tool in tool_list:
        if tool in sentence:
            spoken_tool.append(tool)
            tool_flag=True

    for word in vv_words:
        for bring in bring_list:
            if word == bring:
                bring_flag=True
    if bring_flag is True and tool_flag is True:
        return 0, spoken_tool
    elif bring_flag is True and tool_flag is False:
        return 1, spoken_tool
    else:
        return 2, spoken_tool

with m as source:
    r.adjust_for_ambient_noise(m)
    print("[자바스] 인식을 시작합니다")
    while turn_off_flag==False:
        if recog_flag==True:
            print("[자바스] 듣고있어요")
        audio=r.listen(m,phrase_time_limit=5) # phrase_time_limit = 말을 시작했을때 듣는 최대 시간
        callback(r,audio)

        
        

