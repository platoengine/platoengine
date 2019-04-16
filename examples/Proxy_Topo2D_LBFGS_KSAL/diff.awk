BEGIN {
  rel_tol = 1e-6
  num_total_cols = 18   ## total number of columns in files.  
  num_cols=0
  cols[num_cols++]=3
  cols[num_cols++]=4
  cols[num_cols++]=5
  cols[num_cols++]=6
  cols[num_cols++]=8
  cols[num_cols++]=9
  cols[num_cols++]=10
  passed = 1
  row = 1
}
/^[0-9]/{
  for(i=0; i<num_cols; ++i){
    dif =  $(cols[i])-$(cols[i]+num_total_cols)
    ave = ($(cols[i])+$(cols[i]+num_total_cols))/2.0
    if( ave != 0.0 )
      same = ((dif/ave)^2 < rel_tol^2)
    else 
      same = ((dif)^2 < rel_tol^2)
    if( !same ){
      print "Row ", row, ", col ", cols[i], " -- values are different: ", $(cols[i]), $(cols[i]+num_total_cols)
    }
    passed = passed && same
  }
  row++
}
END {
  if( passed == 1 )
    exit(0)
  else
    exit(1)
}
