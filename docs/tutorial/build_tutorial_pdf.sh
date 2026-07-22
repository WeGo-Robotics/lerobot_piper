#!/usr/bin/env bash
#===============================================================================
# build_tutorial_pdf.sh - PiPER + LeRobot 完整教程 PDF / LaTeX 构建脚本
#===============================================================================
# 功能:
#   1. 按顺序读取教程所有 .md 章节文件
#   2. 组装为完整 HTML 文档（带 LaTeX 风格 CSS）
#   3. 使用 weasyprint 渲染生成 PDF
#   4. 同步生成 .tex LaTeX 源文件
#
# 输出:
#   ../PiPER_LeRobot_Tutorial.pdf    (PDF 文件)
#   ../PiPER_LeRobot_Tutorial.tex    (LaTeX 源文件)
#
# 依赖:
#   python3, python3-markdown, weasyprint (pip)
#===============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="$(realpath "$SCRIPT_DIR/..")"
PDF_OUTPUT="$OUTPUT_DIR/PiPER_LeRobot_Tutorial.pdf"
TEX_OUTPUT="$OUTPUT_DIR/PiPER_LeRobot_Tutorial.tex"
TUTORIAL_DIR="$SCRIPT_DIR"

#===============================================================================
# 预检查：Python3, markdown, weasyprint
#===============================================================================
echo "==> 检查依赖..."

if ! command -v python3 &>/dev/null; then
    echo "错误: 未找到 python3，请安装 Python 3.8+"
    exit 1
fi

echo "   python3: $(python3 --version)"

# 检查 markdown 库
if ! python3 -c "import markdown" &>/dev/null; then
    echo "错误: 未找到 markdown 库，请运行: pip install markdown"
    exit 1
fi
echo "   markdown: $(python3 -c 'import markdown; print(markdown.__version__)')"

# 检查 weasyprint
if ! python3 -c "import weasyprint" &>/dev/null; then
    echo "错误: 未找到 weasyprint 库，请运行: pip install weasyprint"
    exit 1
fi
echo "   weasyprint: $(python3 -c 'import weasyprint; print(weasyprint.__version__)')"

#===============================================================================
# 定义章节文件列表（按阅读顺序）
#===============================================================================
readonly CHAPTERS=(
    "01_system_overview.md"
    "02_hardware_fundamentals.md"
    "03_piper_sdk_deep_dive.md"
    "04_lerobot_framework.md"
    "05_piper_motors_bus.md"
    "06_robot_teleoperator_wrapping.md"
    "07_pc_relay_teleoperation.md"
    "08_data_recording.md"
    "09_imitation_learning_act.md"
    "appendix_a_scripts_reference.md"
    "appendix_b_can_id_reference.md"
    "appendix_c_troubleshooting.md"
)

readonly CHAPTER_TITLES=(
    "系统概述"
    "硬件基础"
    "PiPER SDK 深度解析"
    "LeRobot 框架深度解析"
    "PiPER 电机与总线通信"
    "机器人遥操作封装"
    "PC 中转主从遥操作"
    "数据录制"
    "模仿学习与 ACT 训练"
    "附录 A LeRobot 代码深度解析"
    "附录 B CAN ID 快速参考"
    "附录 C 故障排除指南"
)

#===============================================================================
# 主逻辑：嵌入 Python 脚本完成组装和渲染
#===============================================================================
python3 - "$TUTORIAL_DIR" "$PDF_OUTPUT" "$TEX_OUTPUT" "${CHAPTERS[@]}" << 'PYEOF'
import sys
import os
import re
import textwrap

# ---------------------------------------------------------------------------
# 参数解析
# ---------------------------------------------------------------------------
tutorial_dir = sys.argv[1]
pdf_output   = sys.argv[2]
tex_output   = sys.argv[3]

chapter_files = [
    "01_system_overview.md",
    "02_hardware_fundamentals.md",
    "03_piper_sdk_deep_dive.md",
    "04_lerobot_framework.md",
    "05_piper_motors_bus.md",
    "06_robot_teleoperator_wrapping.md",
    "07_pc_relay_teleoperation.md",
    "08_data_recording.md",
    "09_imitation_learning_act.md",
    "appendix_a_scripts_reference.md",
    "appendix_b_can_id_reference.md",
    "appendix_c_troubleshooting.md",
]

chapter_titles = [
    "系统概述",
    "硬件基础",
    "PiPER SDK 深度解析",
    "LeRobot 框架深度解析",
    "PiPER 电机与总线通信",
    "机器人遥操作封装",
    "PC 中转主从遥操作",
    "数据录制",
    "模仿学习与 ACT 训练",
    "附录 A LeRobot 代码深度解析",
    "附录 B CAN ID 快速参考",
    "附录 C 故障排除指南",
]

# ---------------------------------------------------------------------------
# 导入 markdown
# ---------------------------------------------------------------------------
import markdown
from markdown.extensions import Extension
from markdown.treeprocessors import Treeprocessor
import xml.etree.ElementTree as ET

md = markdown.Markdown(extensions=[
    'fenced_code',
    'tables',
    'codehilite',
    'toc',
])

# ---------------------------------------------------------------------------
# 自定义扩展：收集所有标题用于生成目录
# ---------------------------------------------------------------------------
class HeadingCollector(Treeprocessor):
    def __init__(self):
        super().__init__()
        self.headings = []  # (level, text, anchor_id)  # level: 1 for h1, 2 for h2, 3 for h3

    def run(self, root):
        self.headings = []
        for el in root.iter():
            tag = el.tag.lower() if hasattr(el, 'tag') else ''
            if tag in ('h1', 'h2', 'h3'):
                level = int(tag[1])
                # Generate anchor from text
                text = ET.tostring(el, encoding='unicode', method='text').strip()
                anchor = text.lower().replace(' ', '-')
                # Remove special chars from anchor
                anchor = re.sub(r'[^\w\-]', '', anchor)
                self.headings.append((level, text, anchor))
        return root


class HeadingCollectorExtension(Extension):
    def extendMarkdown(self, md):
        collector = HeadingCollector()
        md.treeprocessors.register(collector, 'heading_collector', 175)
        md.heading_collector = collector


md_with_collector = markdown.Markdown(extensions=[
    'fenced_code',
    'tables',
    'codehilite',
    'toc',
    HeadingCollectorExtension(),
])

# ---------------------------------------------------------------------------
# 读取并转换每个章节
# ---------------------------------------------------------------------------
chapter_html_parts = []
all_headings = []  # (level, text, anchor)
chapter_counter = 1
chapter_numbering = []  # (chapter_num, title, anchor_id)

for i, (fname, chtitle) in enumerate(zip(chapter_files, chapter_titles)):
    fpath = os.path.join(tutorial_dir, fname)
    if not os.path.exists(fpath):
        print(f"   [跳过] 文件不存在: {fname}")
        continue

    with open(fpath, 'r', encoding='utf-8') as f:
        md_text = f.read()

    # 转换 Markdown -> HTML
    # 每次创建新的 markdown 实例以确保 toc 重置
    m = markdown.Markdown(extensions=[
        'fenced_code',
        'tables',
        'codehilite',
        'toc',
        HeadingCollectorExtension(),
    ])
    html_body = m.convert(md_text)

    # 收集标题
    collector = m.heading_collector
    for level, text, anchor in collector.headings:
        # 为 h1 分配章节编号
        is_appendix = fname.startswith('appendix_')
        if level == 1:
            if is_appendix:
                ch_num_str = fname.replace('appendix_', '').upper().replace('.MD', '')
                ch_label = f"{ch_num_str}"
                # Map: appendix_a -> A, appendix_b -> B, appendix_c -> C
                label_map = {'A': 'A', 'B': 'B', 'C': 'C'}
                # Extract letter
                letter = fname.split('_')[1].split('.')[0].upper()
                ch_label = label_map.get(letter, letter.upper())
            else:
                ch_label = str(chapter_counter)
                chapter_counter += 1
            chapter_numbering.append((ch_label, text, anchor))
        all_headings.append((level, text, anchor, ch_label if level == 1 else None))

    # 包裹章节 div
    chapter_div = f'<div class="chapter" id="ch-{i + 1}">\n{html_body}\n</div>\n'
    chapter_html_parts.append(chapter_div)

    print(f"   [已加载] {fname}")

# ---------------------------------------------------------------------------
# 生成目录 HTML
# ---------------------------------------------------------------------------
toc_html_parts = ['<div class="toc">', '<h2>目录</h2>', '<ul class="toc-list">']

for ch_label, ch_title, anchor in chapter_numbering:
    toc_html_parts.append(
        f'  <li class="toc-chapter"><a href="#{anchor}">'
        f'<span class="toc-num">{ch_label}</span> {ch_title}</a></li>'
    )

toc_html_parts.append('</ul>')
toc_html_parts.append('</div>')
toc_html = '\n'.join(toc_html_parts)

# ---------------------------------------------------------------------------
# 组装完整 HTML 文档
# ---------------------------------------------------------------------------
body_html = '\n'.join(chapter_html_parts)

full_html = f'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>PiPER + LeRobot 完整教程</title>
<style>
/* ==============================================================
   LaTeX Article 风格 CSS
   ============================================================== */

@page {{
    size: A4;
    margin: 2.5cm 2cm 2.5cm 2cm;
    @bottom-center {{
        content: "— " counter(page) " —";
        font-family: "Noto Serif CJK SC", "Source Han Serif SC", "STSong", Georgia, serif;
        font-size: 9pt;
        color: #555;
    }}
}}

@page :first {{
    @bottom-center {{
        content: none;
    }}
}}

/* ---- 基础排版 ---- */
body {{
    font-family: "Noto Serif CJK SC", "Source Han Serif SC", "STSong", Georgia, "Times New Roman", serif;
    font-size: 11pt;
    line-height: 1.8;
    color: #1a1a1a;
    max-width: 750px;
    margin: 0 auto;
    padding: 0;
    text-align: justify;
}}

/* ---- 标题页 ---- */
.title-page {{
    page-break-after: always;
    text-align: center;
    padding-top: 180px;
    padding-bottom: 80px;
}}

.title-page h1 {{
    font-size: 28pt;
    font-weight: bold;
    margin-bottom: 20px;
    line-height: 1.3;
    color: #000;
    border: none;
    letter-spacing: 2pt;
}}

.title-page .subtitle {{
    font-size: 14pt;
    color: #555;
    margin-bottom: 60px;
    line-height: 1.5;
}}

.title-page .date-line {{
    font-size: 11pt;
    color: #888;
    margin-top: 40px;
}}

.title-page .meta-info {{
    font-size: 10pt;
    color: #999;
    margin-top: 30px;
    line-height: 2;
}}

/* ---- 目录 ---- */
.toc {{
    page-break-after: always;
    margin-top: 20px;
}}

.toc h2 {{
    font-size: 18pt;
    text-align: center;
    border: none;
    margin-bottom: 30px;
}}

.toc-list {{
    list-style: none;
    padding-left: 0;
}}

.toc-chapter {{
    font-size: 11pt;
    margin-bottom: 8px;
    line-height: 1.6;
}}

.toc-chapter a {{
    text-decoration: none;
    color: #1a1a1a;
}}

.toc-chapter a:hover {{
    text-decoration: underline;
}}

.toc-num {{
    font-weight: bold;
    margin-right: 8px;
    color: #333;
}}

/* ---- 章节标题 ---- */
h1 {{
    font-size: 18pt;
    font-weight: bold;
    margin-top: 40px;
    margin-bottom: 16px;
    line-height: 1.4;
    color: #000;
    border-bottom: 1.5px solid #333;
    padding-bottom: 6px;
    page-break-before: always;
}}

h2 {{
    font-size: 14pt;
    font-weight: bold;
    margin-top: 28px;
    margin-bottom: 12px;
    line-height: 1.4;
    color: #111;
}}

h3 {{
    font-size: 12pt;
    font-weight: bold;
    margin-top: 22px;
    margin-bottom: 8px;
    line-height: 1.4;
    color: #222;
}}

h4 {{
    font-size: 11pt;
    font-weight: bold;
    margin-top: 18px;
    margin-bottom: 6px;
    color: #333;
}}

/* ---- 段落 ---- */
p {{
    margin: 0 0 10px 0;
    text-indent: 0;
}}

/* ---- 强调 ---- */
strong, b {{
    color: #000;
}}

em, i {{
    color: #333;
}}

/* ---- 列表 ---- */
ul, ol {{
    padding-left: 24px;
    margin: 8px 0 14px 0;
}}

li {{
    margin-bottom: 4px;
}}

/* ---- 表格 ---- */
table {{
    width: 100%;
    border-collapse: collapse;
    margin: 16px 0 20px 0;
    font-size: 9.5pt;
}}

thead {{
    background-color: #f0f0f0;
}}

th {{
    border: 1px solid #999;
    padding: 6px 8px;
    font-weight: bold;
    text-align: center;
    background-color: #e8e8e8;
}}

td {{
    border: 1px solid #bbb;
    padding: 5px 8px;
    vertical-align: top;
}}

tr:nth-child(even) {{
    background-color: #fafafa;
}}

/* ---- 代码 ---- */
code {{
    font-family: "Source Code Pro", "Noto Sans Mono CJK SC", "Courier New", "Consolas", monospace;
    font-size: 9pt;
    background-color: #f4f4f4;
    padding: 1px 4px;
    border-radius: 2px;
    color: #333;
}}

pre {{
    font-family: "Source Code Pro", "Noto Sans Mono CJK SC", "Courier New", "Consolas", monospace;
    font-size: 8.5pt;
    background-color: #f5f5f5;
    border: 1px solid #ddd;
    border-left: 3px solid #999;
    padding: 10px 14px;
    margin: 12px 0 16px 0;
    overflow-x: auto;
    line-height: 1.5;
    white-space: pre-wrap;
    word-wrap: break-word;
}}

pre code {{
    background: none;
    padding: 0;
    border-radius: 0;
    font-size: inherit;
    color: #222;
}}

/* ---- 引用块 ---- */
blockquote {{
    border-left: 4px solid #bbb;
    margin: 12px 0;
    padding: 6px 16px;
    background-color: #fafafa;
    color: #444;
    font-style: italic;
}}

/* ---- 水平线 ---- */
hr {{
    border: none;
    border-top: 1px solid #ccc;
    margin: 24px 0;
}}

/* ---- 链接 ---- */
a {{
    color: #2a5db0;
    text-decoration: none;
}}

a:hover {{
    text-decoration: underline;
}}

/* ---- 图片 ---- */
img {{
    max-width: 100%;
    height: auto;
    display: block;
    margin: 12px auto;
}}

/* ---- 章节分页 ---- */
.chapter {{
    page-break-after: always;
}}

.chapter:last-child {{
    page-break-after: auto;
}}

/* ---- 注意/警告框 ---- */
.admonition {{
    border: 1px solid #ccc;
    margin: 14px 0;
    padding: 10px 16px;
    border-radius: 4px;
}}

.admonition.note {{
    border-left: 4px solid #4a90d9;
    background-color: #f0f7ff;
}}

.admonition.warning {{
    border-left: 4px solid #e6a23c;
    background-color: #fef8ee;
}}

.admonition.danger {{
    border-left: 4px solid #f56c6c;
    background-color: #fef0f0;
}}

.admonition.tip {{
    border-left: 4px solid #67c23a;
    background-color: #f0f9eb;
}}

/* ---- 题注 ---- */
figcaption, .caption {{
    font-size: 9.5pt;
    text-align: center;
    color: #666;
    margin-top: 4px;
    margin-bottom: 12px;
}}

/* ---- 打印优化 ---- */
@media print {{
    body {{
        text-align: justify;
    }}

    a {{
        color: #000;
    }}

    pre {{
        white-space: pre-wrap;
    }}
}}
</style>
</head>
<body>

<!-- ============================================================ -->
<!-- 标题页 -->
<!-- ============================================================ -->
<div class="title-page">
<h1>PiPER + LeRobot<br>完整教程</h1>
<div class="subtitle">
从硬件连接到模仿学习部署的端到端指南
</div>
<div class="meta-info">
PiPER 六轴协作机械臂 &times; HuggingFace LeRobot 框架<br>
版本: 1.0<br>
</div>
<div class="date-line">
2025 年 6 月
</div>
</div>

<!-- ============================================================ -->
<!-- 目录 -->
<!-- ============================================================ -->
{toc_html}

<!-- ============================================================ -->
<!-- 正文 -->
<!-- ============================================================ -->
<h1 style="page-break-before: always; border: none; text-align: center; font-size: 20pt;">正文</h1>

{body_html}

</body>
</html>'''

# ---------------------------------------------------------------------------
# 写入临时 HTML 文件，用 weasyprint 渲染 PDF
# ---------------------------------------------------------------------------
import tempfile
import time

html_tmp_path = os.path.join(tempfile.gettempdir(), f'_piper_tutorial_{os.getpid()}.html')
with open(html_tmp_path, 'w', encoding='utf-8') as f:
    f.write(full_html)

print(f"\n==> HTML 已组装 ({len(full_html):,} 字节)")
print(f"   临时文件: {html_tmp_path}")

# ---------------------------------------------------------------------------
# weasyprint 渲染
# ---------------------------------------------------------------------------
print(f"\n==> 使用 weasyprint 生成 PDF...")
from weasyprint import HTML

html_doc = HTML(filename=html_tmp_path)
html_doc.write_pdf(pdf_output)
print(f"   PDF 文件: {pdf_output}")
print(f"   大小: {os.path.getsize(pdf_output):,} 字节")

# ---------------------------------------------------------------------------
# 生成 .tex LaTeX 源文件
# ---------------------------------------------------------------------------
print(f"\n==> 生成 LaTeX 源文件...")

def escape_latex(text):
    """转义 LaTeX 特殊字符"""
    replacements = [
        ('\\', r'\textbackslash{}'),
        ('&', r'\&'),
        ('%', r'\%'),
        ('$', r'\$'),
        ('#', r'\#'),
        ('_', r'\_'),
        ('{', r'\{'),
        ('}', r'\}'),
        ('~', r'\textasciitilde{}'),
        ('^', r'\^{}'),
    ]
    for old, new in replacements:
        text = text.replace(old, new)
    return text


def md_to_latex_simple(md_text):
    """将 Markdown 文本做简单的 LaTeX 转换（不依赖 Pandoc）。
    这是为无法安装 Pandoc 的用户提供的备选方案。"""
    lines = md_text.split('\n')
    latex_lines = []
    in_code_block = False
    in_table = False
    in_list = False

    i = 0
    while i < len(lines):
        line = lines[i]

        # 代码块边界
        if line.strip().startswith('```'):
            if in_code_block:
                latex_lines.append('\\end{lstlisting}')
                in_code_block = False
            else:
                lang = line.strip()[3:].strip() or 'text'
                latex_lines.append(f'\\begin{{lstlisting}}[language={lang}]')
                in_code_block = True
            i += 1
            continue

        if in_code_block:
            latex_lines.append(line)
            i += 1
            continue

        # 表格检测 (简单的 |---| 模式)
        if line.strip().startswith('|') and line.strip().endswith('|'):
            if not in_table:
                latex_lines.append('\\begin{tabular}')
                in_table = True
            # 简单表格处理：导出行内容
            cells = [c.strip() for c in line.strip().strip('|').split('|')]
            latex_lines.append(' & '.join(cells) + ' \\\\')
            if i + 1 < len(lines) and re.match(r'^\|[\s\-:|]+\|$', lines[i + 1].strip()):
                i += 1  # 跳过分隔行
            i += 1
            continue
        elif in_table:
            latex_lines.append('\\end{tabular}')
            in_table = False

        # 标题
        heading_match = re.match(r'^(#{1,6})\s+(.+)$', line)
        if heading_match:
            level = len(heading_match.group(1))
            title = heading_match.group(2)
            cmd_map = {1: 'section', 2: 'subsection', 3: 'subsubsection',
                       4: 'paragraph', 5: 'subparagraph', 6: 'textbf'}
            cmd = cmd_map.get(level, 'textbf')
            if level <= 3:
                latex_lines.append(f'\\{cmd}{{{title}}}')
            else:
                latex_lines.append(f'\\{cmd}{{{title}}}\n')
            i += 1
            continue

        # 无序列表
        if re.match(r'^\s*[\-\*\+]\s+', line):
            latex_lines.append(f'\\item {re.sub(r"^\s*[\-\*\+]\s+", "", line)}')
            i += 1
            continue

        # 有序列表
        ol_match = re.match(r'^\s*(\d+)\.\s+', line)
        if ol_match:
            latex_lines.append(f'\\item[{ol_match.group(1)}] {line[ol_match.end():]}')
            i += 1
            continue

        # 加粗
        line = re.sub(r'\*\*(.+?)\*\*', r'\\textbf{\1}', line)

        # 斜体
        line = re.sub(r'(?<!\*)\*([^\*]+)\*(?!\*)', r'\\textit{\1}', line)

        # 行内代码
        line = re.sub(r'`([^`]+)`', r'\\texttt{\1}', line)

        # 普通段落
        if line.strip():
            latex_lines.append(line)
        else:
            latex_lines.append('')

        i += 1

    # 确保未闭合的代码块被关闭
    if in_code_block:
        latex_lines.append('\\end{lstlisting}')

    return '\n'.join(latex_lines)


# 构建 LaTeX 文档
tex_parts = []
tex_preamble = r'''\documentclass[UTF8,a4paper,11pt]{ctexart}

% ---- 页面设置 ----
\usepackage[top=2.5cm,bottom=2.5cm,left=2cm,right=2cm]{geometry}
\usepackage{setspace}
\onehalfspacing

% ---- 字体 ----
\setCJKmainfont{Noto Serif CJK SC}
\setCJKsansfont{Noto Sans CJK SC}
\setCJKmonofont{Noto Sans Mono CJK SC}

% ---- 代码高亮 ----
\usepackage{listings}
\usepackage{xcolor}

\lstset{
    basicstyle=\ttfamily\small,
    backgroundcolor=\color{gray!10},
    frame=single,
    framesep=8pt,
    rulecolor=\color{gray!40},
    breaklines=true,
    breakatwhitespace=false,
    numbers=none,
    showstringspaces=false,
    columns=flexible,
    keepspaces=true,
    tabsize=4,
    captionpos=b,
    extendedchars=true,
    literate={á}{{\'a}}1 {é}{{\'e}}1 {í}{{\'i}}1 {ó}{{\'o}}1 {ú}{{\'u}}1,
}

% ---- 超链接 ----
\usepackage[colorlinks=true,linkcolor=blue,urlcolor=blue,citecolor=blue]{hyperref}

% ---- 表格 ----
\usepackage{array}
\usepackage{booktabs}

% ---- 图片 ----
\usepackage{graphicx}
\graphicspath{{./images/}}

% ---- 标题设置 ----
\usepackage{titlesec}
\titleformat{\section}{\Large\bfseries}{}{0em}{}
\titleformat{\subsection}{\large\bfseries}{}{0em}{}
\titleformat{\subsubsection}{\normalsize\bfseries}{}{0em}{}

% ---- 页眉页脚 ----
\usepackage{fancyhdr}
\pagestyle{fancy}
\fancyhf{}
\fancyhead[RO,LE]{PiPER + LeRobot 完整教程}
\fancyfoot[RO,LE]{\thepage}
\renewcommand{\headrulewidth}{0.4pt}

% ============================================================
% 文档信息
% ============================================================
\title{PiPER + LeRobot 完整教程}
\author{从硬件连接到模仿学习部署的端到端指南}
\date{2025年6月}

\begin{document}

\maketitle
\thispagestyle{empty}

\newpage
\tableofcontents
\newpage

'''

tex_parts.append(tex_preamble)

# 逐章节追加内容
for i, (fname, chtitle) in enumerate(zip(chapter_files, chapter_titles)):
    fpath = os.path.join(tutorial_dir, fname)
    if not os.path.exists(fpath):
        continue

    with open(fpath, 'r', encoding='utf-8') as f:
        md_text = f.read()

    # 将章节包裹在分组中
    tex_parts.append(f'%% ===== {chtitle} =====')
    tex_parts.append(f'\\begingroup')
    tex_parts.append(md_to_latex_simple(md_text))
    tex_parts.append(f'\\endgroup')
    tex_parts.append(f'\\newpage')
    tex_parts.append('')

tex_parts.append(r'\end{document}')
tex_parts.append('')

tex_content = '\n'.join(tex_parts)

with open(tex_output, 'w', encoding='utf-8') as f:
    f.write(tex_content)

print(f"   LaTeX 文件: {tex_output}")
print(f"   大小: {os.path.getsize(tex_output):,} 字节")

# ---------------------------------------------------------------------------
# 清理临时文件
# ---------------------------------------------------------------------------
try:
    os.unlink(html_tmp_path)
except OSError:
    pass

print(f"\n==> 构建完成!")
print(f"   PDF:    {pdf_output}")
print(f"   LaTeX:  {tex_output}")
print(f"\n提示: 若要编译 LaTeX 源文件为 PDF，请运行:")
print(f"   cd {os.path.dirname(tex_output)}")
print(f"   xelatex {os.path.basename(tex_output)}")
print(f"   xelatex {os.path.basename(tex_output)}  # 运行两次以生成目录")

PYEOF

#===============================================================================
# 完成
#===============================================================================
echo ""
echo "脚本执行完毕。输出文件:"
echo "  PDF:   $PDF_OUTPUT"
echo "  LaTeX: $TEX_OUTPUT"
