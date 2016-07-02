void storetolog(const char *msg)
{
    sprintf(tempstr,"curl -u gym:muskler http://intranet.strongest.se/gympassv4/index.php/DoorpassV5/storetolog?passagepoint=%s&msg=%s", PASSAGE_POINT, msg);
    Console.println(tempstr);

    p.runShellCommandAsynchronously(tempstr);
}

void str_append(char *s, char c) 
{
     int len = strlen(s);
     s[len] = c;
     s[len+1] = '\0';
}

void urlencode(char* dest, char* src)
{
  int i;
  
  strcpy(dest,"");
  
  for (i=0; i<strlen(src); i++)
  {
    switch(src[i])
    {
      case ' ':
        strcat(dest, "%20");
        break;

      case '!':
        strcat(dest, "%21");
        break;

      case '"':
        strcat(dest, "%22");
        break;

      case '#':
        strcat(dest, "%23");
        break;

      case '$':
        strcat(dest, "%24");
        break;

      case '%':
        strcat(dest, "%25");
        break;

      case '&':
        strcat(dest, "%26");
        break;

      case '\'':
        strcat(dest, "%27");
        break;

      case '(':
        strcat(dest, "%28");
        break;

      case ')':
        strcat(dest, "%29");
        break;

      case '*':
        strcat(dest, "%2A");
        break;

      case '+':
        strcat(dest, "%2B");
        break;

      case ',':
        strcat(dest, "%2C");
        break;

      case '-':
        strcat(dest, "%2D");
        break;

      case '.':
        strcat(dest, "%2E");
        break;

      case '/':
        strcat(dest, "%2F");
        break;

      case ':':
        strcat(dest, "%3A");
        break;

      case ';':
        strcat(dest, "%3B");
        break;

      case '<':
        strcat(dest, "%3C");
        break;

      case '=':
        strcat(dest, "%3D");
        break;

      case '>':
        strcat(dest, "%3E");
        break;

      case '?':
        strcat(dest, "%3F");
        break;

      case '@':
        strcat(dest, "%40");
        break;

      case '{':
        strcat(dest, "%7B");
        break;

      case '|':
        strcat(dest, "%7C");
        break;

      case '}':
        strcat(dest, "%7D");
        break;

      default:
        str_append(dest, src[i]);
        break;
    }
  }
}

