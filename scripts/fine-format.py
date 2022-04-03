from json import dump
import re
import sys

file_name = sys.argv[1]

output_file = "format-latex.txt"

# formatting rules

sr_end = {r"\n": r"", 
          r"\;": r"",
          r"\,": r"",
          r"\mathrm{}": r""}

sr_trig_to_alg = {r"\cos": r"c_",
                  r"\sin": r"s_"
                  }

sr_left_right = {r"\left": r"",
                 r"\right": r""}

sr_theta_gone = {r"q_{1}": r"1",
                r"q_{2}": r"2",
                r"q_{3}": r"3"}

sr_theta_gone_2 = {r"q_1": r"1",
                r"q_2": r"2",
                r"q_3": r"3"}

sr_arr_4 = {r"(\begin{array}{cccc}": r"\begin{bmatrix}",
          r"\end{array})": r"\end{bmatrix}"}

sr_vec = {r"(\begin{array}{c}": r"\begin{bmatrix}",
          r"\end{array})": r"\end{bmatrix}"}

sr_arr_3 = {r"(\begin{array}{ccc}": r"\begin{bmatrix}",
          r"\end{array})": r"\end{bmatrix}"}

# re.sub(r"\\theta_(\d)", r"\1", )

f = open(output_file, "w")
f.close()

with open(file_name) as f:
    for line in f:
       
        new_text = line
        
        # spaces
        new_text = re.sub(r"( )", r"", new_text)
        
        for key, val in sr_end.items():
            new_text = new_text.replace(key, val)
        
        for key, val in sr_trig_to_alg.items():
            new_text = new_text.replace(key, val)
            
        # new_text = line
        for key, val in sr_left_right.items():
            new_text = new_text.replace(key, val)
            
        for key, val in sr_theta_gone.items():
            new_text = new_text.replace(key, val)
            
        for key, val in sr_theta_gone_2.items():
            new_text = new_text.replace(key, val)
            
        for key, val in sr_arr_4.items():
            new_text = new_text.replace(key, val)
            
        for key, val in sr_vec.items():
            new_text = new_text.replace(key, val)
        
        for key, val in sr_arr_3.items():
            new_text = new_text.replace(key, val)
            

        # scalar multiplication
        new_text = re.sub(r"(\d)\.(\d)(\d)", r"\1.\2*\3", new_text)
        
        # sub numbers arithmetic
        new_text = re.sub(r"\((\d)\.(\d)\*(\d)\)", r"{\1.\2*\3}", new_text)
        new_text = re.sub(r"\((\d)\.(\d)\*(\d)\+(\d)\)", r"{\1.\2*\3+\4}", new_text)
        
        # to remove single digit parantheses    
        new_text = re.sub(r"\((\d)\)", r"\1",new_text)
        
        # to remove addition in subscript
        new_text = re.sub(r"\((\d)\+(\d)\ \)", r"{\1\2}", new_text)
        
        # to remove addition in subscript
        new_text = re.sub(r"\((\d)\+(\d)\)", r"{\1\2}", new_text)
        
        new_text = re.sub(r"\((\d)\ \)", r"\1", new_text)
        
        
        print(new_text)
        output = {"old": line, "new_text": new_text}
        with open(output_file, "a") as fp:
            fp.write(new_text)


# r and \ infront of 1


# re.sub(r"\\theta_(\d)", r"\1", )

# new_text = old_text
# for key, val in search_and_replace.items():
#     new_text = new_text.replace(key, val)


# output = {"old": old_text, "new_text": new_text}
# with open("tmp.txt", "w") as fp:
#     fp.write(new_text)
