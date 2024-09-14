import re

def extract_numbers(filename, keyword):
    # 打开文件并读取文本内容
    with open(filename, 'r') as file:
        content = file.read()

    # 定义一个空数组，用于存储文本中包含指定词汇的数字
    numbers = []

    # 使用正则表达式在文本中查找指定词汇及其后面的数字，并将数字添加到数组中
    pattern = re.compile(keyword + r'\s*(\d+)')
    matches = pattern.findall(content)
    for match in matches:
        numbers.append(int(match))


    # 返回排序后的数组
    return numbers


numbers = extract_numbers('C:\Users\12911\OneDrive\桌面\Neues Textdokument (2).txt', 'FORCE: ')
print(numbers)