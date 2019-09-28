###########################################################
#
#   FILENAME:       ra_filter.py
#
#   DESCRIPTION:    Running Average Filter
#
#
###########################################################

class RA_Filter:
    def __init__(self, num_terms):
        self.terms = []
        for i in range(0, num_terms):
            self.terms.append(0)

    def filter(self, value):
        # Rotate Back Terms
        for i in range(0, len(self.terms) - 1):
            self.terms[i] = self.terms[i+1]

        self.terms[-1] = value

        #Sum up terms
        sum = 0
        for i in range(0, len(self.terms)):
            sum += self.terms[i]

        return sum / len(self.terms)