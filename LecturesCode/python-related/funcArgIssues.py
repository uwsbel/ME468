def argListAspects(intArg, listArg):
    intArg = 2  # now the name intArg points to object "2"
    listArg.append('d') # the mutable list object changed here
    print('Inside the function:', intArg, listArg)

def main():
    i = 1
    theList = ['a', 'b' , 'c']
    print('Before function call:', i, theList)

    # call function; some args (the mutables) get changed, some not
    argListAspects(i, theList)
    print('After function call:', i, theList)

main()