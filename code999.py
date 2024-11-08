    # Main loop
    while (mov11 != dectheta3shiftedlr or mov12 != dectheta1shiftedlr or mov13 != dectheta3shiftedrr or mov14 != dectheta1shiftedrr or mov15 != decthetaHipshiftedrr or mov16 != decgammar or mov21 != dectheta3shiftedll or mov22 != dectheta1shiftedll or mov23 != dectheta3shiftedrl or mov24 != dectheta1shiftedrl or mov25 != decthetaHipshiftedrl or mov26 != decgammal):

        moveMotor()

        # Increment variables as needed
        if mov11 != dectheta3shiftedlr:
            mov11 += 1
        if mov12 != dectheta1shiftedlr:
            mov12 += 1
        if mov13 != dectheta3shiftedrr:
            mov13 += 1
        if mov14 != dectheta1shiftedrr:
            mov14 += 1
        if mov15 != decthetaHipshiftedrr:
            mov15 += 1
        if mov16 != decgammar:
            mov16 += 1
        if mov21 != dectheta3shiftedll:
            mov21 += 1
        if mov22 != dectheta1shiftedll:
            mov22 += 1
        if mov23 != dectheta3shiftedrl:
            mov23 += 1
        if mov24 != dectheta1shiftedrl:
            mov24 += 1
        if mov25 != decthetaHipshiftedrl:
            mov25 += 1
        if mov26 != decgammal:
            mov26 += 1
