from openai import OpenAI
from secret import OPENAI_API_KEY


client = OpenAI(api_key=OPENAI_API_KEY, base_url="https://openai.batalov.me/v1")



messages = []

def prompt(system_prompt, user_prompt):
    global messages
    if not messages:
        messages.append({
            "role": "system",
            "content": system_prompt,
        })
    messages += [
        {
            "role": "user",
            "content": user_prompt,
        }
    ]
    completion = client.chat.completions.create(
        model="gpt-4o",
        messages=messages
    )
    messages.append(completion.choices[0].message.to_dict())
    # print(messages)
    return completion.choices[0].message.content


if __name__ == "__main__":
    while True:
        user_prompt = input()
        print(prompt(user_prompt))
