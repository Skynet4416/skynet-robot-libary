using Plots;

rectangle(w, h, x, y) = Shape(x-(w/2) .+ [0,w,w,0], y-(h/2) .+ [0,0,h,h])

x_array = [0.34338715221616106, 0.6853240997697687, 1.0075817046952178, 1.3163601721934743, 1.6147264864729942, 1.9052796559258878, 2.1900477135430387, 2.4705232910519777, 2.7476424345125334, 3.0217587613812156, 3.2927507363215454, 3.5603484038861146, 3.8244328214446446, 4.085120337229668, 4.342741349922815, 4.597800869729275, 4.850936075771671, 5.102873486800261]


y_array = [0.9233420156393742, 1.651534858352609, 2.2619488512607795, 2.762481058120718, 3.1575472241991998, 3.4505751752533365, 3.6440245556792834, 3.7395503602450146, 3.738224051650695, 3.6409160815221986, 3.4488617856464012, 3.1641205067067895, 2.789630015501094, 2.329044394301272, 1.7865735483712137, 1.166843847235232, 0.47476764094877055, -0.2845785086607474]
x_array_runge_kutta = [0.20689024882024123, 0.41378049764048247, 0.5844284642165294, 0.7181093029034982, 0.8266829440870417, 0.920072230565974, 1.0036896302032057, 1.0802786761098044, 1.1513917707092984, 1.218040993601287, 1.2809527331334096, 1.3406771883202624, 1.3976443192527848, 1.4521968138292805, 1.5046113018781153, 1.555112760523173, 1.6038846762806358, 1.651076455202775, 1.6968090166869987, 1.7411791890877377, 1.7842633259621035, 1.826120423253288, 1.8667949130826487, 1.906319229064045, 1.9447161794372616, 1.9820011278114729, 2.018183964678579, 2.0532708510365496, 2.087265722206029, 2.120171549794585, 2.151991369211337, 2.1827290874895993, 2.21239009106886, 2.2409816759150907, 2.2685133234260406, 2.2949968454583556, 2.3204464209038673, 2.3448785448036626, 2.3683119092092104, 2.390767233014868, 2.4122670558974, 2.432835509387105, 2.4524980760285993, 2.471281345621663, 2.4892127757063847, 2.506320461802711, 2.522632921451473, 2.5381788948403363, 2.552987163733021, 2.567086389545146, 2.5805049707111882, 2.593270918946649, 2.6054117536079153, 2.6169544130691476, 2.627925181851025, 2.6383496321317574, 2.6482525782296165]


y_array_runge_kutta  = [1.0668902488202412, 1.2737804976404825, 1.4405056642165295, 1.5662624469034983, 1.663470318482846, 1.7423737412818128, 1.8085124449983285, 1.8647011054131253, 1.9125488528158572, 1.9531134206450338, 1.9871557966439484, 2.0152514670381074, 2.0378487722186875, 2.055304244695694, 2.067906029041926, 2.075890372752754, 2.079453808033207, 2.078762535392576, 2.073959921314429, 2.0651726649721245, 2.0525159605472463, 2.036097832315209, 2.0160227270095863, 1.9923944002796148, 1.965318120907688, 1.9349022256946171, 1.9012590769544362, 1.8645054932936818, 1.8247627370256252, 1.7821561465625237, 1.7368145004435558, 1.688869193309274, 1.6384532950929231, 1.5857005544493172, 1.5307443969504178, 1.473716958420974, 1.4147481843005063, 1.3539650172933808, 1.291490687922165, 1.227444115996022, 1.1619394254707678, 1.095085570699001, 1.0269860686019592, 0.9577388287641937, 0.887436071759126, 0.8161643250436678, 0.7440044853914911, 0.6710319369452792, 0.5973167144426823, 0.5229237019038634, 0.44791285796930125, 0.37233946006890817, 0.2962543606266774, 0.2197042495125302, 0.14273191791115486, 0.06537651966382048, -0.012326173060121653]   
plot(x_array,y_array,label="magnus recursion",linecolor=:blue)
plot!(x_array_runge_kutta,y_array_runge_kutta,label="runge kutta",linecolor=:red)
plot!(rectangle(1.22,0.05,4.0,2.7178),opacity = 0.5)

# polyfit(x_array_lift_recursion,y_array_lift_recursion)
# plot!(x_array_drag_recursion,y_array_drag_recursion,label="drag recursion",linecolor=:red)
# plot!(x_array_drag_no_recursion,y_array_drag_no_recursion,label="drag no recursion",linecolor=:orange)
# plot!(x_array_lift_no_recursion,y_array_lift_no_recursion,label="magnus no recursion",linecolor=:black)
# plot!(x_array_changing_lift_recursion,y_array_changing_lift_recursion,label="magnus recursion",linecolor=:pink)
# plot!(x_array_nasa_lift_recursion,y_array_nasa_lift_recursion,label="magnus recursion",linecolor=:pink)


savefig("test"); print("Done\n")