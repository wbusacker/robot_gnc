###########################################################
#
#   FILENAME:       iir_filter.py
#
#   DESCRIPTION:    Infinite Impulse Response Filter
#
#
###########################################################

class IIR_Filter:
    def __init__(self):
        self.forward_terms  = [0, 0]
        self.back_terms     = [0, 0]
        self.input_gain     = 1.031426266
        self.back_coeff     = -0.9390625058

    def filter(self, value):

        # Rotate Back Terms
        self.back_terms[1]      = self.back_terms[0]
        self.forward_terms[1]   = self.forward_terms[0]

        # Calculate new terms
        self.forward_terms[0]   = value / self.input_gain
        self.back_terms[0]      =  (self.forward_terms[1] + 
                                    self.forward_terms[0] + 
                                    ( self.back_coeff * self.back_terms[1])
                                   )

        return self.back_terms[0]


if __name__ == '__main__':
    test_filter = IIR_Filter()

