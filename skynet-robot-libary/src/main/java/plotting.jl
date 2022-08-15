using Plots;

rectangle(w, h, x, y) = Shape(x-(w/2) .+ [0,w,w,0], y-(h/2) .+ [0,0,h,h])
x_array_lift_recursion = [0.989914513919027, 2.1315041427449377, 3.0958569677901457, 3.968964257966024, 4.762764909319172, 5.493245955782973, 6.1725902908377295, 6.810618776383123, 7.415238541885005, 7.992807694819462, 8.548401129734355, 9.086006208705784, 9.608673132152516, 10.11864693801704, 10.617510765064099, 11.106362134300282, 11.586016081917297, 12.05719622847602, 12.520671811710784, 12.977327694051262, 13.42818101650054, 13.874364819096195, 14.317094508245631, 14.757627537329475, 15.19722280483336, 15.637103741284967, 16.078427310716194, 16.52225991070525, 16.969560285053117, 17.421168987311873]


y_array_lift_recursion = [1.4540505554343968, 2.8111010462362835, 4.015945856255289, 5.079632153110505, 6.013536437250273, 6.825743249251346, 7.523163430922437, 8.111445315118463, 8.59518836052866, 8.978097581838288, 9.263131504746719, 9.452654854272083, 9.5486114043384, 9.552728610544992, 9.4667512838416, 9.292674558267484, 9.032923586853837, 8.69043838156964, 8.268665135365037, 7.771487774309529, 7.203133651614443, 6.5680732740739165, 5.8709234505157735, 5.116358402888827, 4.309031257356565, 3.453507134341158, 2.55420820129575, 1.6153704049214126, 0.6410111299992315, -0.3650932783513565]





x_array = [0.0, 1.0547564341377398, 2.092675966115981, 3.041175378986554, 3.9227093561509485, 4.748921983137454, 5.529470670465323, 6.272234638662715, 6.983588012226804, 7.668660149909827, 8.331556529788891, 8.975563944978818, 9.603349001259248, 10.217148382754495, 10.81893908251601, 11.41057336740606, 11.993868875673623, 12.570654294276814, 13.14277892777486, 13.712097438151025, 14.280440338459803, 14.849578605814303]


y_array = [0.1, 0.8891552582510377, 1.5668441305854461, 2.1635976102711494, 2.6732130037544444, 3.0972052326446744, 3.43646112495812, 3.6919377474517376, 3.8646168504073932, 3.955539616736759, 3.9658761351943306, 3.897015807995552, 3.7506606639800397, 3.5289002597509507, 3.23425088596571, 2.8696528701330832, 2.4384318329986336, 1.9442362607694361, 1.3909636466778434, 0.7826840631372549, 0.12356653881938079, -0.5821887935012863]
plot(x_array_lift_recursion,y_array_lift_recursion,label="magnus recursion",linecolor=:blue)
plot!(x_array,y_array,label="magnus recursion2",linecolor=:red)

plot!(rectangle(1.22,0.05,12,2.7178),opacity = 0.5)
# polyfit(x_array_lift_recursion,y_array_lift_recursion)
# plot!(x_array_drag_recursion,y_array_drag_recursion,label="drag recursion",linecolor=:red)
# # plot!(x_array_drag_no_recursion,y_array_drag_no_recursion,label="drag no recursion",linecolor=:orange)
# # plot!(x_array_lift_no_recursion,y_array_lift_no_recursion,label="magnus no recursion",linecolor=:black)
# plot!(x_array_changing_lift_recursion,y_array_changing_lift_recursion,label="magnus recursion",linecolor=:pink)
# plot!(x_array_nasa_lift_recursion,y_array_nasa_lift_recursion,label="magnus recursion",linecolor=:pink)


savefig("test"); print("Done\n")
