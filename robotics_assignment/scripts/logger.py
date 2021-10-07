# -*- coding: utf-8 -*-
# @Author: Franklin Selva
# @Date:   2021-09-28 21:24:07
# @Last Modified by:   Franklin Selva
# @Last Modified time: 2021-10-07 21:33:25
#!/usr/bin/env python
import os
import yaml
import json

from importlib import import_module

import rospy
import rospkg
from rospy import topics
from std_msgs.msg import Bool

STATUS: bool = False
MSG: dict = {}


def status_callback(msg):
    global STATUS
    STATUS = msg.data


class CallbackMethod:
    msg = None

    def __call__(self, *args, **kwds):
        return self.msg

    def callback(self, msg):
        self.msg = msg


class TopicDataHolder:
    topic: str = ""
    data: dict = {}
    callback: CallbackMethod

    def __init__(self, topic: str, data: dict, callback: CallbackMethod) -> None:
        self.topic = topic
        self.data = data
        self.callback = callback


def parse_yaml_file():
    topic_names = {}
    with open(
        os.path.join(
            rospkg.RosPack().get_path("robotics_assignment"), "config/topics.yaml"
        )
    ) as f:
        rospy.loginfo("Importing message modules")
        data = yaml.load(f, Loader=yaml.FullLoader)
        topics = data.keys()
        for topic in topics:
            msg_type = data[topic]["type"]
            topic_name = data[topic]["ns"] + data[topic]["topic_name"]
            module = msg_type.split("/")
            rospy.loginfo(f"Importing message type: {msg_type}")
            try:
                MSG[f"{module[1]}"] = getattr(
                    import_module(f"{module[0]}.msg"), f"{module[1]}"
                )

                topic_names[topic_name] = {f"type": f"{msg_type}", f"data": []}
            except ImportError:
                rospy.logwarn(f"Unable to import module for message type: {msg_type}")

    return topic_names


def save_json(choice: str, handler: TopicDataHolder):
    rospy.loginfo(f"Saving Recorded messages: {choice}")
    file_name = choice.replace("/", "_") + ".json"
    save_path = os.path.join(
        rospkg.RosPack().get_path("robotics_assignment"),
        "data",
        file_name,
    )
    print(handler.topic)
    print(type(handler.data), handler.data)

    with open(save_path, "w+") as f:
        json.dump(handler.data, f, indent=4)
        rospy.loginfo(f"File saved at: {save_path}")


def main():
    """Main Function"""
    rospy.init_node("logger_node", anonymous=False)
    subscriber = rospy.Subscriber("log_mode_status", Bool, status_callback)
    rate = rospy.Rate(60)

    global STATUS
    PRINT_MSG: bool = True
    SAVE: bool = True
    topic_handler: list[TopicDataHolder] = []

    topic_data = parse_yaml_file()
    for topic in topic_data:
        CALLBACK = CallbackMethod()
        class_name = topic_data[topic]["type"].split("/")[1]
        rospy.Subscriber(topic, MSG[class_name], callback=CALLBACK.callback)
        topic_handler.append(TopicDataHolder(topic, topic_data[topic], CALLBACK))

    choice = ""
    while not rospy.is_shutdown():
        if STATUS and PRINT_MSG:
            print("Choose a topic from below list")
            for i, topic in enumerate(topic_data):
                print(f"{i}. {topic}")
            selection = int(input("> "))
            for i, topic in enumerate(topic_data):
                if i == selection:
                    choice = topic
            rospy.loginfo(f"Started Recording messages: {choice}")
            PRINT_MSG = False
        if not STATUS and SAVE:
            for handler in topic_handler:
                if handler.topic == choice:
                    save_json(choice, handler)
            PRINT_MSG = True
            rate.sleep()

        for handler in topic_handler:
            if handler.topic == choice:
                if STATUS:
                    handler.data["data"].append(handler.callback.msg)
                    SAVE = True


if __name__ == "__main__":
    main()
