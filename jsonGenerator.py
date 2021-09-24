#!/usr/bin/python
import sys


def parseByWord(dataInput=[], matchList=[str, []], matchStatic=[], blackList=[]):
    matchingSignal = False
    matchesHeader = []
    matchesTail = []
    matchesData = []
    matchStatus = []
    matchBuffer = [0, 0]
    matchCount = 0
    for line in data:
        line = line.strip()
        matchBuffer.append(line)
        if len(matchBuffer) >= 2:
            matchBuffer.pop(0)
        # match statement 1
        if matchBuffer[0] == matchList[0]:
            matchesHeader.append(matchBuffer[0])
            matchingSignal = True

        if matchingSignal:
            if matchBuffer[1] == matchStatic[0]:
                matchesHeader.append(matchBuffer[0])
                matchStatus.append(True)
                matchesData.append([])
                matchCount += 1
            elif matchBuffer[0] == matchStatic[0]:
                pass
            elif matchBuffer[1][0:len(matchStatic[1])] == matchStatic[1]:
                if len(matchStatus) == 1:
                    matchesTail.append(matchBuffer[1])
                    matchStatus.pop()
                    return matchesData, matchesHeader, matchesTail
                elif matchStatus[len(matchStatus) - 1] == True:
                    matchesTail.append(matchBuffer[1])
                    matchesData[matchCount-1].append(matchBuffer[0])
                    matchStatus.pop()
            elif matchBuffer[0][0:len(matchStatic[1])] == matchStatic[1]:
                pass
            else:
                for x in blackList:
                    if matchBuffer[0][0:len(x)] != x:
                        if matchBuffer[0] != '':
                            matchesData[matchCount - 1].append(matchBuffer[0])


data = []
with open("src/SingleAPM.hpp", 'r') as tojsonfile:
    data = tojsonfile.readlines()

main, header, tail = parseByWord(data, ["struct APMSettinngs", [
                                 "none"]], matchStatic=["{", "}"], blackList=["//"])

jsonlist = []
jsonlistR = []
for x in range(len(main)):
    for s in range(len(main[x])):
        jsonlist.append(main[x][s].split(" ")[1].split(";")[0])

i = 0
n = 0
for x in main:
    if len(x) != 0:
        for s in x:
            jsonlistR.append(("p." + tail[i].split(" ")
                              [1].split(";")[0] + '.' + jsonlist[n].split(";")[0]))
            n += 1
        i += 1
print("JSONLIST:")
print(jsonlist)
print("JSONLISTR:")
print(jsonlistR)

for s in range(len(jsonlist)):
    print('{"'+jsonlist[s]+'",'+jsonlistR[s]+"},")
for s in range(len(jsonlist)):
    print('j.at("'+jsonlist[s]+'").get_to('+jsonlistR[s]+");")
