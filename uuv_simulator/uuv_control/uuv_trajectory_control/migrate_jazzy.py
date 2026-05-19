import os
from pathlib import Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

ROOT = "."

PROMPT = """

Rules:

1. rospy -> rclpy

2. Publishers:
rospy.Publisher → node.create_publisher()

3. Subscribers:
rospy.Subscriber → node.create_subscription()

4. Services:
rospy.Service → node.create_service()

5. ServiceProxy → node.create_client()

6. rospy.get_param → declare_parameter/get_parameter

7. tf -> tf2_ros

8. Remove catkin dependencies

9. Preserve logic exactly

10. Output ONLY converted code.
"""

for root, dirs, files in os.walk(ROOT):

    for f in files:

        if f.endswith(".py"):

            path = Path(root) / f

            print("Converting:", path)

            try:
                code = path.read_text(encoding="utf-8", errors="ignore")

                response = ollama.chat(
                    model="qwen2.5-coder:14b",
                    messages=[
                        {
                            "role": "system",
                            "content": PROMPT
                        },
                        {
                            "role": "user",
                            "content": code
                        }
                    ]
                )

                new_code = response["message"]["content"]

                path.write_text(new_code, encoding="utf-8")

            except Exception as e:
                print("FAILED:", path, e)

print("DONE")

